// Package viamorbslam3 implements simultaneous localization and mapping
// This is an Experimental package
package viamorbslam3

import (
	"bufio"
	"context"
	"io"
	"log"
	"os"
	"path/filepath"
	"strconv"
	"strings"
	"sync"
	"time"

	"github.com/edaniels/golog"
	"github.com/pkg/errors"
	"go.opencensus.io/trace"
	pb "go.viam.com/api/service/slam/v1"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/components/generic"
	"go.viam.com/rdk/config"
	"go.viam.com/rdk/registry"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/rimage/transform"
	"go.viam.com/rdk/services/slam"
	"go.viam.com/rdk/services/slam/grpchelper"
	"go.viam.com/rdk/spatialmath"
	rdkutils "go.viam.com/rdk/utils"
	slamConfig "go.viam.com/slam/config"
	"go.viam.com/slam/dataprocess"
	slamSensorUtils "go.viam.com/slam/sensors/utils"
	slamUtils "go.viam.com/slam/utils"
	goutils "go.viam.com/utils"
	"go.viam.com/utils/pexec"
	"golang.org/x/exp/slices"
)

var (
	cameraValidationMaxTimeoutSec = 30 // reconfigurable for testing
	dialMaxTimeoutSec             = 30 // reconfigurable for testing
	// Model specifies the unique resource-triple across the rdk.
	Model             = resource.NewModel("viam", "slam", "orbslamv3")
	supportedSubAlgos = []SubAlgo{Mono, Rgbd}
)

const (
	defaultDataRateMsec         = 200
	defaultMapRateSec           = 60
	cameraValidationIntervalSec = 1.
	parsePortMaxTimeoutSec      = 60
	// time format for the slam service.
	opTimeoutErrorMessage = "bad scan: OpTimeout"
	localhost0            = "localhost:0"
	// DefaultExecutableName is what this program expects to call to start the grpc server.
	DefaultExecutableName = "orb_grpc_server"
)

// SubAlgo defines the ORB_SLAM3 specific algorithms that we support.
type SubAlgo string

const (
	// Mono represents monocular vision (uses only one camera).
	Mono SubAlgo = "mono"
	// Rgbd uses a color camera and a depth camera for SLAM.
	Rgbd SubAlgo = "rgbd"
)

// SetCameraValidationMaxTimeoutSecForTesting sets cameraValidationMaxTimeoutSec for testing.
func SetCameraValidationMaxTimeoutSecForTesting(val int) {
	cameraValidationMaxTimeoutSec = val
}

// SetDialMaxTimeoutSecForTesting sets dialMaxTimeoutSec for testing.
func SetDialMaxTimeoutSecForTesting(val int) {
	dialMaxTimeoutSec = val
}

func init() {
	registry.RegisterService(slam.Subtype, Model, registry.Service{
		Constructor: func(ctx context.Context, deps registry.Dependencies, c config.Service, logger golog.Logger) (interface{}, error) {
			return New(ctx, deps, c, logger, false, DefaultExecutableName)
		},
	})
    /*
	config.RegisterServiceAttributeMapConverter(slam.Subtype, Model, func(attributes config.AttributeMap) (interface{}, error) {
		var attrs slamConfig.AttrConfig
		decoder, err := mapstructure.NewDecoder(&mapstructure.DecoderConfig{TagName: "json", Result: &attrs})
		if err != nil {
			return nil, err
		}
		if err := decoder.Decode(attributes); err != nil {
			return nil, err
		}
		return &attrs, nil
	}, &slamConfig.AttrConfig{})
    */
   config.RegisterComponentAttributeMapConverter(
		slam.Subtype,
		Model,
		func(attributes config.AttributeMap) (interface{}, error) {
			var conf slamConfig.AttrConfig
            log.Println("ZACK register component attr map converter")
            log.Println(attributes)
			return config.TransformAttributeMapToStruct(&conf, attributes)
		},
		&slamConfig.AttrConfig{})
}

// runtimeServiceValidation ensures the service's data processing and saving is valid for the mode and
// cameras given.
func runtimeServiceValidation(
	ctx context.Context,
	cams []camera.Camera,
	orbSvc *orbslamService,
) error {
	if !orbSvc.useLiveData {
		return nil
	}

	var err error
	paths := make([]string, 0, 1)
	startTime := time.Now()

	for {
		var currPaths []string
		currPaths, err = orbSvc.getAndSaveData(ctx, cams)
		paths = append(paths, currPaths...)

		if err == nil {
			break
		}

		// This takes about 5 seconds, so the timeout should be sufficient.
		if time.Since(startTime) >= time.Duration(cameraValidationMaxTimeoutSec)*time.Second {
			return errors.Wrap(err, "error getting data in desired mode")
		}
		if !goutils.SelectContextOrWait(ctx, cameraValidationIntervalSec*time.Second) {
			return ctx.Err()
		}
	}

	// Generate a new yaml file based off the camera configuration and presence of maps
	if err = orbSvc.orbGenYAML(ctx, cams[0]); err != nil {
		return errors.Wrap(err, "error generating .yaml config")
	}

	for _, path := range paths {
		if err := os.RemoveAll(path); err != nil {
			return errors.Wrap(err, "error removing generated file during validation")
		}
	}

	return nil
}

// orbslamService is the structure of the ORB_SLAM3 slam service.
type orbslamService struct {
	generic.Unimplemented
	primarySensorName string
	subAlgo           SubAlgo
	executableName    string // by default: DefaultExecutableName
	slamProcess       pexec.ProcessManager
	clientAlgo        pb.SLAMServiceClient
	clientAlgoClose   func() error

	configParams        map[string]string
	dataDirectory       string
	deleteProcessedData bool
	useLiveData         bool

	port       string
	dataRateMs int
	mapRateSec int

	dev bool

	cancelFunc              func()
	logger                  golog.Logger
	activeBackgroundWorkers sync.WaitGroup

	bufferSLAMProcessLogs        bool
	slamProcessLogReader         io.ReadCloser
	slamProcessLogWriter         io.WriteCloser
	slamProcessBufferedLogReader bufio.Reader
}

// configureCameras will check the config to see if any cameras are desired and if so, grab the cameras from
// the robot. We assume there are at most two cameras and that we only require intrinsics from the first one.
// Returns the name of the first camera.
func configureCameras(ctx context.Context,
	svcConfig *slamConfig.AttrConfig,
	deps registry.Dependencies,
	logger golog.Logger,
) (string, []camera.Camera, error) {
	if len(svcConfig.Sensors) > 0 {
		logger.Debug("Running in live mode")
		cams := make([]camera.Camera, 0, len(svcConfig.Sensors))
		// The first camera is expected to be RGB.
		primarySensorName := svcConfig.Sensors[0]
		cam, err := camera.FromDependencies(deps, primarySensorName)
		if err != nil {
			return "", nil, errors.Wrapf(err, "error getting camera %v for slam service", primarySensorName)
		}
		proj, err := cam.Projector(ctx)
		if err != nil {
			return "", nil, errors.Wrap(err,
				"Unable to get camera features for first camera, make sure the color camera is listed first")
		}

		intrinsics, ok := proj.(*transform.PinholeCameraIntrinsics)
		if !ok {
			return "", nil, transform.NewNoIntrinsicsError("Intrinsics do not exist")
		}

		err = intrinsics.CheckValid()
		if err != nil {
			return "", nil, err
		}

		props, err := cam.Properties(ctx)
		if err != nil {
			return "", nil, errors.Wrap(err, "error getting camera properties for slam service")
		}

		brownConrady, ok := props.DistortionParams.(*transform.BrownConrady)
		if !ok {
			return "", nil, errors.New("error getting distortion_parameters for slam service, " +
				"only BrownConrady distortion parameters are supported")
		}
		if err := brownConrady.CheckValid(); err != nil {
			return "", nil, errors.Wrapf(err, "error validating distortion_parameters for slam service")
		}

		cams = append(cams, cam)

		// If there is a second camera, it is expected to be depth.
		if len(svcConfig.Sensors) > 1 {
			depthCameraName := svcConfig.Sensors[1]
			logger.Debugf("Two cameras found for slam service, assuming %v is for color and %v is for depth",
				primarySensorName, depthCameraName)
			depthCam, err := camera.FromDependencies(deps, depthCameraName)
			if err != nil {
				return "", nil, errors.Wrapf(err, "error getting camera %v for slam service", depthCameraName)
			}
			cams = append(cams, depthCam)
		}

		return primarySensorName, cams, nil
	}
	return "", nil, nil
}

// GetPosition forwards the request for positional data to the slam library's gRPC service. Once a response is received,
// it is unpacked into a Pose and a component reference string.
func (orbSvc *orbslamService) GetPosition(ctx context.Context, name string) (spatialmath.Pose, string, error) {
	ctx, span := trace.StartSpan(ctx, "viamorbslam3::orbslamService::GetPosition")
	defer span.End()

	req := &pb.GetPositionRequest{Name: name}

	resp, err := orbSvc.clientAlgo.GetPosition(ctx, req)
	if err != nil {
		return nil, "", errors.Wrap(err, "error getting SLAM position")
	}
	pose := spatialmath.NewPoseFromProtobuf(resp.GetPose())
	componentReference := resp.GetComponentReference()
	returnedExt := resp.Extra.AsMap()

	return slamUtils.CheckQuaternionFromClientAlgo(pose, componentReference, returnedExt)
}

// GetPointCloudMapStream creates a request, calls the slam algorithms GetPointCloudMapStream endpoint and returns a callback
// function which will return the next chunk of the current pointcloud map.
func (orbSvc *orbslamService) GetPointCloudMapStream(ctx context.Context, name string) (func() ([]byte, error), error) {
	ctx, span := trace.StartSpan(ctx, "viamorbslam3::orbslamService::GetPointCloudMapStream")
	defer span.End()

	return grpchelper.GetPointCloudMapCallback(ctx, name, orbSvc.clientAlgo)
}

// GetInternalStateStream creates a request, calls the slam algorithms GetInternalStateStream endpoint and returns a callback
// function which will return the next chunk of the current internal state of the slam algo.
func (orbSvc *orbslamService) GetInternalStateStream(ctx context.Context, name string) (func() ([]byte, error), error) {
	ctx, span := trace.StartSpan(ctx, "viamorbslam3::orbslamService::GetInternalStateStream")
	defer span.End()

	return grpchelper.GetInternalStateCallback(ctx, name, orbSvc.clientAlgo)
}

// GetPointCloudMap creates a request, calls the slam algorithms GetPointCloudMap endpoint and returns a callback
// function which will return the next chunk of the current pointcloud map.
func (orbSvc *orbslamService) GetPointCloudMap(ctx context.Context, name string) (func() ([]byte, error), error) {
	ctx, span := trace.StartSpan(ctx, "viamorbslam3::orbslamService::GetPointCloudMap")
	defer span.End()

	return grpchelper.GetPointCloudMapCallback(ctx, name, orbSvc.clientAlgo)
}

// GetInternalState creates a request, calls the slam algorithms GetInternalState endpoint and returns a callback
// function which will return the next chunk of the current internal state of the slam algo.
func (orbSvc *orbslamService) GetInternalState(ctx context.Context, name string) (func() ([]byte, error), error) {
	ctx, span := trace.StartSpan(ctx, "viamorbslam3::orbslamService::GetInternalState")
	defer span.End()

	return grpchelper.GetInternalStateCallback(ctx, name, orbSvc.clientAlgo)
}

// New returns a new slam service for the given robot.
func New(ctx context.Context,
	deps registry.Dependencies,
	config config.Service,
	logger golog.Logger,
	bufferSLAMProcessLogs bool,
	executableName string,
) (slam.Service, error) {
	ctx, span := trace.StartSpan(ctx, "viamorbslam3::New")
	defer span.End()

	svcConfig, ok := config.ConvertedAttributes.(*slamConfig.AttrConfig)
	if !ok {
		return nil, rdkutils.NewUnexpectedTypeError(svcConfig, config.ConvertedAttributes)
	}

	primarySensorName, cams, err := configureCameras(ctx, svcConfig, deps, logger)
	if err != nil {
		return nil, errors.Wrap(err, "configuring camera error")
	}

	subAlgo := SubAlgo(svcConfig.ConfigParams["mode"])
	if !slices.Contains(supportedSubAlgos, subAlgo) {
		return nil, errors.Errorf("%v does not have a mode %v",
			config.Model.Name, svcConfig.ConfigParams["mode"])
	}

	if err = slamConfig.SetupDirectories(svcConfig.DataDirectory, logger); err != nil {
		return nil, errors.Wrap(err, "unable to setup working directories")
	}

	var directoryNames []string
	if subAlgo == Rgbd {
		directoryNames = []string{"rgb", "depth"}
	} else if subAlgo == Mono {
		directoryNames = []string{"rgb"}
	}
	for _, directoryName := range directoryNames {
		directoryPath := filepath.Join(svcConfig.DataDirectory, "data", directoryName)
		if _, err := os.Stat(directoryPath); os.IsNotExist(err) {
			logger.Warnf("%v directory does not exist", directoryPath)
			if err := os.Mkdir(directoryPath, os.ModePerm); err != nil {
				return nil, errors.Errorf("issue creating directory at %v: %v", directoryPath, err)
			}
		}
	}

	port, dataRateMsec, mapRateSec, useLiveData, deleteProcessedData, err := slamConfig.GetOptionalParameters(
		svcConfig,
		localhost0,
		defaultDataRateMsec,
		defaultMapRateSec,
		logger,
	)
	if err != nil {
		return nil, err
	}
	cancelCtx, cancelFunc := context.WithCancel(ctx)

	// SLAM Service Object
	orbSvc := &orbslamService{
		primarySensorName:     primarySensorName,
		subAlgo:               subAlgo,
		executableName:        executableName,
		slamProcess:           pexec.NewProcessManager(logger),
		configParams:          svcConfig.ConfigParams,
		dataDirectory:         svcConfig.DataDirectory,
		useLiveData:           useLiveData,
		deleteProcessedData:   deleteProcessedData,
		port:                  port,
		dataRateMs:            dataRateMsec,
		mapRateSec:            mapRateSec,
		cancelFunc:            cancelFunc,
		logger:                logger,
		bufferSLAMProcessLogs: bufferSLAMProcessLogs,
		dev:                   svcConfig.Dev,
	}

	var success bool
	defer func() {
		if !success {
			if err := orbSvc.Close(); err != nil {
				logger.Errorw("error closing out after error", "error", err)
			}
		}
	}()

	if err := runtimeServiceValidation(cancelCtx, cams, orbSvc); err != nil {
		return nil, errors.Wrap(err, "runtime slam service error")
	}

	orbSvc.StartDataProcess(cancelCtx, cams, nil)

	if err := orbSvc.StartSLAMProcess(ctx); err != nil {
		return nil, errors.Wrap(err, "error with slam service slam process")
	}

	client, clientClose, err := slamConfig.SetupGRPCConnection(ctx, orbSvc.port, dialMaxTimeoutSec, logger)
	if err != nil {
		return nil, errors.Wrap(err, "error with initial grpc client to slam algorithm")
	}
	orbSvc.clientAlgo = client
	orbSvc.clientAlgoClose = clientClose

	success = true
	return orbSvc, nil
}

// Close closes out of all slam-related processes.
func (orbSvc *orbslamService) Close() error {
	defer func() {
		if orbSvc.clientAlgoClose != nil {
			goutils.UncheckedErrorFunc(orbSvc.clientAlgoClose)
		}
	}()
	orbSvc.cancelFunc()
	if orbSvc.bufferSLAMProcessLogs {
		if orbSvc.slamProcessLogReader != nil {
			if err := orbSvc.slamProcessLogReader.Close(); err != nil {
				return errors.Wrap(err, "error occurred during closeout of slam log reader")
			}
		}
		if orbSvc.slamProcessLogWriter != nil {
			if err := orbSvc.slamProcessLogWriter.Close(); err != nil {
				return errors.Wrap(err, "error occurred during closeout of slam log writer")
			}
		}
	}
	if err := orbSvc.StopSLAMProcess(); err != nil {
		return errors.Wrap(err, "error occurred during closeout of process")
	}
	orbSvc.activeBackgroundWorkers.Wait()
	return nil
}

// TODO 05/10/2022: Remove from SLAM service once GRPC data transfer is available.
// StartDataProcess starts the background control loop for sending data from the camera(s) to the data directory for processing.
func (orbSvc *orbslamService) StartDataProcess(
	cancelCtx context.Context,
	cams []camera.Camera,
	c chan int,
) {
	if !orbSvc.useLiveData {
		return
	}

	orbSvc.activeBackgroundWorkers.Add(1)
	if err := cancelCtx.Err(); err != nil {
		if !errors.Is(err, context.Canceled) {
			orbSvc.logger.Errorw("unexpected error in SLAM service", "error", err)
		}
		orbSvc.activeBackgroundWorkers.Done()
		return
	}
	goutils.PanicCapturingGo(func() {
		ticker := time.NewTicker(time.Millisecond * time.Duration(orbSvc.dataRateMs))
		defer ticker.Stop()
		defer orbSvc.activeBackgroundWorkers.Done()

		for {
			if err := cancelCtx.Err(); err != nil {
				if !errors.Is(err, context.Canceled) {
					orbSvc.logger.Errorw("unexpected error in SLAM data process", "error", err)
				}
				return
			}

			select {
			case <-cancelCtx.Done():
				return
			case <-ticker.C:
				orbSvc.activeBackgroundWorkers.Add(1)
				if err := cancelCtx.Err(); err != nil {
					if !errors.Is(err, context.Canceled) {
						orbSvc.logger.Errorw("unexpected error in SLAM service", "error", err)
					}
					orbSvc.activeBackgroundWorkers.Done()
					return
				}
				goutils.PanicCapturingGo(func() {
					defer orbSvc.activeBackgroundWorkers.Done()
					if _, err := orbSvc.getAndSaveData(cancelCtx, cams); err != nil {
						orbSvc.logger.Warn(err)
					}
					if c != nil {
						c <- 1
					}
				})
			}
		}
	})
}

// GetSLAMProcessConfig returns the process config for the SLAM process.
func (orbSvc *orbslamService) GetSLAMProcessConfig() pexec.ProcessConfig {
	var args []string

	args = append(args, "-sensors="+orbSvc.primarySensorName)
	args = append(args, "-config_param="+slamUtils.DictToString(orbSvc.configParams))
	args = append(args, "-data_rate_ms="+strconv.Itoa(orbSvc.dataRateMs))
	args = append(args, "-map_rate_sec="+strconv.Itoa(orbSvc.mapRateSec))
	args = append(args, "-data_dir="+orbSvc.dataDirectory)
	args = append(args, "-delete_processed_data="+strconv.FormatBool(orbSvc.deleteProcessedData))
	args = append(args, "-use_live_data="+strconv.FormatBool(orbSvc.useLiveData))
	args = append(args, "-port="+orbSvc.port)
	args = append(args, "--aix-auto-update")

	return pexec.ProcessConfig{
		ID:      "slam_orbslamv3",
		Name:    orbSvc.executableName,
		Args:    args,
		Log:     true,
		OneShot: false,
	}
}

func (orbSvc *orbslamService) GetSLAMProcessBufferedLogReader() bufio.Reader {
	return orbSvc.slamProcessBufferedLogReader
}

// StartSLAMProcess starts up the SLAM library process by calling the executable binary and giving it the necessary arguments.
func (orbSvc *orbslamService) StartSLAMProcess(ctx context.Context) error {
	ctx, span := trace.StartSpan(ctx, "viamorbslam3::orbslamService::StartSLAMProcess")
	defer span.End()

	processConfig := orbSvc.GetSLAMProcessConfig()

	var logReader io.ReadCloser
	var logWriter io.WriteCloser
	var bufferedLogReader bufio.Reader
	if orbSvc.port == localhost0 || orbSvc.bufferSLAMProcessLogs {
		logReader, logWriter = io.Pipe()
		bufferedLogReader = *bufio.NewReader(logReader)
		processConfig.LogWriter = logWriter
	}

	_, err := orbSvc.slamProcess.AddProcessFromConfig(ctx, processConfig)
	if err != nil {
		return errors.Wrap(err, "problem adding slam process")
	}

	orbSvc.logger.Debug("starting slam process")

	if err = orbSvc.slamProcess.Start(ctx); err != nil {
		return errors.Wrap(err, "problem starting slam process")
	}

	if orbSvc.port == localhost0 {
		timeoutCtx, timeoutCancel := context.WithTimeout(ctx, parsePortMaxTimeoutSec*time.Second)
		defer timeoutCancel()

		if !orbSvc.bufferSLAMProcessLogs {
			defer func(logger golog.Logger) {
				if err := logReader.Close(); err != nil {
					logger.Debugw("Closing logReader returned an error", "error", err)
				}
			}(orbSvc.logger)
			defer func(logger golog.Logger) {
				if err := logWriter.Close(); err != nil {
					logger.Debugw("Closing logReader returned an error", "error", err)
				}
			}(orbSvc.logger)
		}

		for {
			if err := timeoutCtx.Err(); err != nil {
				return errors.Wrapf(err, "error getting port from slam process")
			}

			line, err := bufferedLogReader.ReadString('\n')
			if err != nil {
				return errors.Wrapf(err, "error getting port from slam process")
			}
			portLogLinePrefix := "Server listening on "
			if strings.Contains(line, portLogLinePrefix) {
				linePieces := strings.Split(line, portLogLinePrefix)
				if len(linePieces) != 2 {
					return errors.Errorf("failed to parse port from slam process log line: %v", line)
				}
				orbSvc.port = "localhost:" + strings.TrimRight(linePieces[1], "\n")
				break
			}
		}
	}

	if orbSvc.bufferSLAMProcessLogs {
		orbSvc.slamProcessLogReader = logReader
		orbSvc.slamProcessLogWriter = logWriter
		orbSvc.slamProcessBufferedLogReader = bufferedLogReader
	}

	return nil
}

// StopSLAMProcess uses the process manager to stop the created slam process from running.
func (orbSvc *orbslamService) StopSLAMProcess() error {
	if err := orbSvc.slamProcess.Stop(); err != nil {
		return errors.Wrap(err, "problem stopping slam process")
	}
	return nil
}

// getAndSaveData implements the data extraction for saving to the directory path (data subfolder) specified in
// the config. It returns the full filepath for each file saved along with any error associated with the data creation or saving.
func (orbSvc *orbslamService) getAndSaveData(
	ctx context.Context,
	cams []camera.Camera,
) ([]string, error) {
	ctx, span := trace.StartSpan(ctx, "viamorbslam3::orbslamService::getAndSaveDataSparse")
	defer span.End()

	switch orbSvc.subAlgo {
	case Mono:
		if len(cams) != 1 {
			return nil, errors.Errorf("expected 1 camera for mono slam, found %v", len(cams))
		}

		image, release, err := slamSensorUtils.GetPNGImage(ctx, cams[0])
		if release != nil {
			defer release()
		}
		if err != nil {
			if err.Error() == opTimeoutErrorMessage {
				orbSvc.logger.Warnw("Skipping this scan due to error", "error", err)
				return nil, nil
			}
			return nil, err
		}
		filenames, err := createTimestampFilenames(orbSvc.dataDirectory, orbSvc.primarySensorName, ".png", orbSvc.subAlgo)
		if err != nil {
			return nil, err
		}

		filename := filenames[0]
		return []string{filename}, dataprocess.WriteBytesToFile(image, filename)
	case Rgbd:
		if len(cams) != 2 {
			return nil, errors.Errorf("expected 2 cameras for Rgbd slam, found %v", len(cams))
		}

		images, releaseFuncs, err := orbSvc.getSimultaneousColorAndDepth(ctx, cams)
		for _, rFunc := range releaseFuncs {
			if rFunc != nil {
				defer rFunc()
			}
		}
		if err != nil {
			if err.Error() == opTimeoutErrorMessage {
				orbSvc.logger.Warnw("Skipping this scan due to error", "error", err)
				return nil, nil
			}
			return nil, err
		}

		filenames, err := createTimestampFilenames(orbSvc.dataDirectory, orbSvc.primarySensorName, ".png", orbSvc.subAlgo)
		if err != nil {
			return nil, err
		}
		for i, filename := range filenames {
			if err = dataprocess.WriteBytesToFile(images[i], filename); err != nil {
				return filenames, err
			}
		}
		return filenames, nil
	default:
		return nil, errors.Errorf("invalid subAlgo %v specified", orbSvc.subAlgo)
	}
}

// getSimultaneousColorAndDepth gets the color and depth images from the cameras as close to simultaneously as possible.
func (orbSvc *orbslamService) getSimultaneousColorAndDepth(
	ctx context.Context,
	cams []camera.Camera,
) ([2][]byte, [2]func(), error) {
	var wg sync.WaitGroup
	var images [2][]byte
	var releaseFuncs [2]func()
	var errs [2]error

	for i := 0; i < 2; i++ {
		orbSvc.activeBackgroundWorkers.Add(1)
		wg.Add(1)
		if err := ctx.Err(); err != nil {
			if !errors.Is(err, context.Canceled) {
				orbSvc.logger.Errorw("unexpected error in SLAM service", "error", err)
			}
			orbSvc.activeBackgroundWorkers.Done()
			return images, releaseFuncs, err
		}
		iLoop := i
		goutils.PanicCapturingGo(func() {
			defer orbSvc.activeBackgroundWorkers.Done()
			defer wg.Done()
			images[iLoop], releaseFuncs[iLoop], errs[iLoop] = slamSensorUtils.GetPNGImage(ctx, cams[iLoop])
		})
	}
	wg.Wait()

	for _, err := range errs {
		if err != nil {
			return images, releaseFuncs, err
		}
	}

	return images, releaseFuncs, nil
}

// createTimestampFilenames creates a file for camera data with the specified sensor name and timestamp written into the filename.
// For RGBD cameras, two filenames are created with the same timestamp in different directories.
func createTimestampFilenames(dataDirectory, primarySensorName, fileType string, subAlgo SubAlgo) ([]string, error) {
	timeStamp := time.Now()
	dataDir := filepath.Join(dataDirectory, "data")
	rbgDataDir := filepath.Join(dataDir, "rgb")
	depthDataDir := filepath.Join(dataDir, "depth")

	switch subAlgo {
	case Mono:
		rgbFilename := dataprocess.CreateTimestampFilename(rbgDataDir, primarySensorName, fileType, timeStamp)
		return []string{rgbFilename}, nil
	case Rgbd:
		rgbFilename := dataprocess.CreateTimestampFilename(rbgDataDir, primarySensorName, fileType, timeStamp)
		depthFilename := dataprocess.CreateTimestampFilename(depthDataDir, primarySensorName, fileType, timeStamp)
		return []string{rgbFilename, depthFilename}, nil
	default:
		return nil, errors.Errorf("Invalid sub algo: %v", subAlgo)
	}
}
