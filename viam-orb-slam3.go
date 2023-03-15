// Package viamorbslam3 implements simultaneous localization and mapping
// This is an Experimental package
package viamorbslam3

import (
	"bufio"
	"bytes"
	"context"
	"image"
	"image/jpeg"
	"io"
	"os"
	"path/filepath"
	"strconv"
	"strings"
	"sync"
	"time"

	"github.com/edaniels/golog"
	"github.com/edaniels/gostream"
	"github.com/mitchellh/mapstructure"
	"github.com/pkg/errors"
	"go.opencensus.io/trace"
	v1 "go.viam.com/api/common/v1"
	pb "go.viam.com/api/service/slam/v1"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/components/generic"
	"go.viam.com/rdk/config"
	pc "go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/registry"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/rimage"
	"go.viam.com/rdk/rimage/transform"
	"go.viam.com/rdk/services/slam"
	"go.viam.com/rdk/spatialmath"
	rdkutils "go.viam.com/rdk/utils"
	"go.viam.com/rdk/vision"
	slamConfig "go.viam.com/slam/config"
	"go.viam.com/slam/dataprocess"
	slamUtils "go.viam.com/slam/utils"
	goutils "go.viam.com/utils"
	"go.viam.com/utils/pexec"
	"go.viam.com/utils/protoutils"

	"github.com/viamrobotics/viam-orb-slam3/internal/grpchelper"
)

var (
	cameraValidationMaxTimeoutSec = 30 // reconfigurable for testing
	dialMaxTimeoutSec             = 30 // reconfigurable for testing
    Model = resource.NewModel("viam","slam","orbslamv3")
)

const (
	defaultDataRateMsec         = 200
	defaultMapRateSec           = 60
	cameraValidationIntervalSec = 1.
	parsePortMaxTimeoutSec      = 60
	// time format for the slam service.
	opTimeoutErrorMessage = "bad scan: OpTimeout"
	localhost0            = "localhost:0"
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
				return New(ctx, deps, c, logger, false)
			},
		})
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

	// TODO 05/05/2022: This will be removed once GRPC data transfer is available as the responsibility for
	// calling the right algorithms (Next vs NextPointCloud) will be held by the slam libraries themselves
	// Note: if GRPC data transfer is delayed to after other algorithms (or user custom algos) are being
	// added this point will be revisited
	for {
		switch orbSvc.slamLib.AlgoType {
		case slam.Sparse:
			var currPaths []string
			currPaths, err = orbSvc.getAndSaveDataSparse(ctx, cams)
			paths = append(paths, currPaths...)
		case slam.Dense:
			var path string
			path, err = orbSvc.getAndSaveDataDense(ctx, cams)
			paths = append(paths, path)
		default:
			return errors.Errorf("invalid slam algorithm %q", orbSvc.slamLib.AlgoName)
		}

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

	// For ORBSLAM, generate a new yaml file based off the camera configuration and presence of maps
	if strings.Contains(orbSvc.slamLib.AlgoName, "orbslamv3") {
		if err = orbSvc.orbGenYAML(ctx, cams[0]); err != nil {
			return errors.Wrap(err, "error generating .yaml config")
		}
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
	slamLib           slam.LibraryMetadata
	subAlgo           slam.Mode
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
			if len(svcConfig.Sensors) == 1 {
				logger.Debug("No camera features found, please make sure to specify both a color and depth camera or a rgbd camera")
			} else {
				return "", nil, errors.Wrap(err,
					"Unable to get camera features for first camera, make sure the color camera is listed first")
			}
		} else {
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

// Position forwards the request for positional data to the slam library's gRPC service. Once a response is received,
// it is unpacked into a PoseInFrame.
func (slamSvc *orbslamService) Position(ctx context.Context, name string, extra map[string]interface{}) (*referenceframe.PoseInFrame, error) {
	ctx, span := trace.StartSpan(ctx, "slam::orbslamService::Position")
	defer span.End()

	ext, err := protoutils.StructToStructPb(extra)
	if err != nil {
		return nil, err
	}

	var pInFrame *referenceframe.PoseInFrame
	var returnedExt map[string]interface{}
    var componentReference string

	// TODO: Once RSDK-1053 (https://viam.atlassian.net/browse/RSDK-1066) is complete the original code before extracting position
	// from GetPosition will be removed and the GetPositionNew -> GetPosition
	if slamSvc.dev {
		slamSvc.logger.Debug("IN DEV MODE (position request)")
		req := &pb.GetPositionNewRequest{Name: name}

		resp, err := slamSvc.clientAlgo.GetPositionNew(ctx, req)
		if err != nil {
			return nil, errors.Wrap(err, "error getting SLAM position")
		}

        componentReference = resp.GetComponentReference()
		pInFrame = referenceframe.NewPoseInFrame(componentReference, spatialmath.NewPoseFromProtobuf(resp.GetPose()))
		returnedExt = resp.Extra.AsMap()
	} else {
		//nolint:staticcheck
		req := &pb.GetPositionRequest{Name: name, Extra: ext}

		resp, err := slamSvc.clientAlgo.GetPosition(ctx, req)
		if err != nil {
			return nil, errors.Wrap(err, "error getting SLAM position")
		}

        componentReference = resp.Pose.ReferenceFrame;
		pInFrame = referenceframe.ProtobufToPoseInFrame(resp.Pose)
		returnedExt = resp.Extra.AsMap()
	}

    newPose, _, err := slamUtils.CheckQuaternionFromClientAlgo(pInFrame.Pose(), componentReference, returnedExt);
    pInFrame = referenceframe.NewPoseInFrame(pInFrame.Parent(), newPose)

	return pInFrame, nil
}

// GetPosition forwards the request for positional data to the slam library's gRPC service. Once a response is received,
// it is unpacked into a Pose and a component reference string.
func (slamSvc *orbslamService) GetPosition(ctx context.Context, name string) (spatialmath.Pose, string, error) {
	ctx, span := trace.StartSpan(ctx, "slam::orbslamService::GetPosition")
	defer span.End()

	req := &pb.GetPositionNewRequest{Name: name}

	resp, err := slamSvc.clientAlgo.GetPositionNew(ctx, req)
	if err != nil {
		return nil, "", errors.Wrap(err, "error getting SLAM position")
	}
	pose := spatialmath.NewPoseFromProtobuf(resp.GetPose())
	componentReference := resp.GetComponentReference()
	returnedExt := resp.Extra.AsMap()

	return slamUtils.CheckQuaternionFromClientAlgo(pose, componentReference, returnedExt)
}

// GetMap forwards the request for map data to the slam library's gRPC service. Once a response is received it is unpacked
// into a mimeType and either a vision.Object or image.Image.
func (slamSvc *orbslamService) GetMap(
	ctx context.Context,
	name, mimeType string,
	cp *referenceframe.PoseInFrame,
	include bool,
	extra map[string]interface{},
) (
	string, image.Image, *vision.Object, error,
) {
	ctx, span := trace.StartSpan(ctx, "slam::orbslamService::GetMap")
	defer span.End()

	var cameraPosition *v1.Pose
	if cp != nil {
		cameraPosition = referenceframe.PoseInFrameToProtobuf(cp).Pose
	}

	ext, err := protoutils.StructToStructPb(extra)
	if err != nil {
		return "", nil, nil, err
	}

	var imData image.Image
	var vObj *vision.Object

	// TODO: Once RSDK-1053 (https://viam.atlassian.net/browse/RSDK-1066) is complete the original code that extracts
	// the map will be removed and GetMap will be changed to GetPointCloudMap
	if slamSvc.dev {
		slamSvc.logger.Debug("IN DEV MODE (map request)")

		//nolint:staticcheck
		reqPCMap := &pb.GetPointCloudMapRequest{
			Name: name,
		}

		if mimeType != rdkutils.MimeTypePCD {
			return "", nil, nil, errors.New("non-pcd return type is impossible in while in dev mode")
		}

		//nolint:staticcheck
		resp, err := slamSvc.clientAlgo.GetPointCloudMap(ctx, reqPCMap)
		if err != nil {
			return "", imData, vObj, errors.Errorf("error getting SLAM map (%v) : %v", mimeType, err)
		}
		pointcloudData := resp.GetPointCloudPcd()
		if pointcloudData == nil {
			return "", nil, nil, errors.New("get map read pointcloud unavailable")
		}
		pc, err := pc.ReadPCD(bytes.NewReader(pointcloudData))
		if err != nil {
			return "", nil, nil, errors.Wrap(err, "get map read pointcloud failed")
		}

		vObj, err = vision.NewObject(pc)
		if err != nil {
			return "", nil, nil, errors.Wrap(err, "get map creating vision object failed")
		}

		mimeType = rdkutils.MimeTypePCD
	} else {
		//nolint:staticcheck
		req := &pb.GetMapRequest{
			Name:               name,
			MimeType:           mimeType,
			CameraPosition:     cameraPosition,
			IncludeRobotMarker: include,
			Extra:              ext,
		}

		//nolint:staticcheck
		resp, err := slamSvc.clientAlgo.GetMap(ctx, req)
		if err != nil {
			return "", imData, vObj, errors.Errorf("error getting SLAM map (%v) : %v", mimeType, err)
		}

		switch mimeType {
		case rdkutils.MimeTypeJPEG:
			imData, err = jpeg.Decode(bytes.NewReader(resp.GetImage()))
			if err != nil {
				return "", nil, nil, errors.Wrap(err, "get map decode image failed")
			}
		case rdkutils.MimeTypePCD:
			pointcloudData := resp.GetPointCloud()
			if pointcloudData == nil {
				return "", nil, nil, errors.New("get map read pointcloud unavailable")
			}
			pc, err := pc.ReadPCD(bytes.NewReader(pointcloudData.PointCloud))
			if err != nil {
				return "", nil, nil, errors.Wrap(err, "get map read pointcloud failed")
			}

			vObj, err = vision.NewObject(pc)
			if err != nil {
				return "", nil, nil, errors.Wrap(err, "get map creating vision object failed")
			}
		}
		mimeType = resp.MimeType
	}

	return mimeType, imData, vObj, nil
}

// GetInternalState forwards the request for the SLAM algorithms's internal state. Once a response is received, it is returned
// to the user.
func (slamSvc *orbslamService) GetInternalState(ctx context.Context, name string) ([]byte, error) {
	ctx, span := trace.StartSpan(ctx, "slam::orbslamService::GetInternalState")
	defer span.End()

	//nolint:staticcheck
	req := &pb.GetInternalStateRequest{Name: name}

	//nolint:staticcheck
	resp, err := slamSvc.clientAlgo.GetInternalState(ctx, req)
	if err != nil {
		return nil, errors.Wrap(err, "error getting the internal state from the SLAM client")
	}

	internalState := resp.GetInternalState()
	return internalState, err
}

// GetPointCloudMapStream creates a request, calls the slam algorithms GetPointCloudMapStream endpoint and returns a callback
// function which will return the next chunk of the current pointcloud map.
func (slamSvc *orbslamService) GetPointCloudMapStream(ctx context.Context, name string) (func() ([]byte, error), error) {
	ctx, span := trace.StartSpan(ctx, "slam::orbslamService::GetPointCloudMapStream")
	defer span.End()

	return grpchelper.GetPointCloudMapStreamCallback(ctx, name, slamSvc.clientAlgo)
}

// GetInternalStateStream creates a request, calls the slam algorithms GetInternalStateStream endpoint and returns a callback
// function which will return the next chunk of the current internal state of the slam algo.
func (slamSvc *orbslamService) GetInternalStateStream(ctx context.Context, name string) (func() ([]byte, error), error) {
	ctx, span := trace.StartSpan(ctx, "slam::orbslamService::GetInternalStateStream")
	defer span.End()

	return grpchelper.GetInternalStateStreamCallback(ctx, name, slamSvc.clientAlgo)
}

// New returns a new slam service for the given robot.
func New(ctx context.Context,
    deps registry.Dependencies, 
    config config.Service, 
    logger golog.Logger, 
    bufferSLAMProcessLogs bool,
) (slam.Service, error) {
	ctx, span := trace.StartSpan(ctx, "slam::slamService::New")
	defer span.End()

	svcConfig, ok := config.ConvertedAttributes.(*slamConfig.AttrConfig)
	if !ok {
		return nil, rdkutils.NewUnexpectedTypeError(svcConfig, config.ConvertedAttributes)
	}

	primarySensorName, cams, err := configureCameras(ctx, svcConfig, deps, logger)
	if err != nil {
		return nil, errors.Wrap(err, "configuring camera error")
	}

	modelName := string(config.Model.Name)
	slamLib, ok := slam.SLAMLibraries[modelName]
	if !ok {
		return nil, errors.Errorf("%v algorithm specified not in implemented list", modelName)
	}

	subAlgo, ok := slamLib.SlamMode[svcConfig.ConfigParams["mode"]]
	if !ok {
		return nil, errors.Errorf("getting data with specified algorithm %v, and desired mode %v",
			modelName, svcConfig.ConfigParams["mode"])
	}

	if err = slamConfig.SetupDirectories(svcConfig.DataDirectory, logger); err != nil {
		return nil, errors.Wrap(err, "unable to setup working directories")
	}

	if subAlgo == slam.Rgbd || subAlgo == slam.Mono {
		var directoryNames []string
		if subAlgo == slam.Rgbd {
			directoryNames = []string{"rgb", "depth"}
		} else if subAlgo == slam.Mono {
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
	slamSvc := &orbslamService{
		primarySensorName:     primarySensorName,
		slamLib:               slam.SLAMLibraries[string(config.Model.Name)],
		subAlgo:               subAlgo,
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
			if err := slamSvc.Close(); err != nil {
				logger.Errorw("error closing out after error", "error", err)
			}
		}
	}()

	if err := runtimeServiceValidation(cancelCtx, cams, slamSvc); err != nil {
		return nil, errors.Wrap(err, "runtime slam service error")
	}

	slamSvc.StartDataProcess(cancelCtx, cams, nil)

	if err := slamSvc.StartSLAMProcess(ctx); err != nil {
		return nil, errors.Wrap(err, "error with slam service slam process")
	}

	client, clientClose, err := slamConfig.SetupGRPCConnection(ctx, slamSvc.port, dialMaxTimeoutSec, logger)
	if err != nil {
		return nil, errors.Wrap(err, "error with initial grpc client to slam algorithm")
	}
	slamSvc.clientAlgo = client
	slamSvc.clientAlgoClose = clientClose

	success = true
	return slamSvc, nil
}

// Close out of all slam related processes.
func (slamSvc *orbslamService) Close() error {
	defer func() {
		if slamSvc.clientAlgoClose != nil {
			goutils.UncheckedErrorFunc(slamSvc.clientAlgoClose)
		}
	}()
	slamSvc.cancelFunc()
	if slamSvc.bufferSLAMProcessLogs {
		if slamSvc.slamProcessLogReader != nil {
			if err := slamSvc.slamProcessLogReader.Close(); err != nil {
				return errors.Wrap(err, "error occurred during closeout of slam log reader")
			}
		}
		if slamSvc.slamProcessLogWriter != nil {
			if err := slamSvc.slamProcessLogWriter.Close(); err != nil {
				return errors.Wrap(err, "error occurred during closeout of slam log writer")
			}
		}
	}
	if err := slamSvc.StopSLAMProcess(); err != nil {
		return errors.Wrap(err, "error occurred during closeout of process")
	}
	slamSvc.activeBackgroundWorkers.Wait()
	return nil
}

// TODO 05/10/2022: Remove from SLAM service once GRPC data transfer is available.
// startDataProcess is the background control loop for sending data from camera to the data directory for processing.
func (slamSvc *orbslamService) StartDataProcess(
	cancelCtx context.Context,
	cams []camera.Camera,
	c chan int,
) {
	if !slamSvc.useLiveData {
		return
	}

	slamSvc.activeBackgroundWorkers.Add(1)
	if err := cancelCtx.Err(); err != nil {
		if !errors.Is(err, context.Canceled) {
			slamSvc.logger.Errorw("unexpected error in SLAM service", "error", err)
		}
		slamSvc.activeBackgroundWorkers.Done()
		return
	}
	goutils.PanicCapturingGo(func() {
		ticker := time.NewTicker(time.Millisecond * time.Duration(slamSvc.dataRateMs))
		defer ticker.Stop()
		defer slamSvc.activeBackgroundWorkers.Done()

		for {
			if err := cancelCtx.Err(); err != nil {
				if !errors.Is(err, context.Canceled) {
					slamSvc.logger.Errorw("unexpected error in SLAM data process", "error", err)
				}
				return
			}

			select {
			case <-cancelCtx.Done():
				return
			case <-ticker.C:
				slamSvc.activeBackgroundWorkers.Add(1)
				if err := cancelCtx.Err(); err != nil {
					if !errors.Is(err, context.Canceled) {
						slamSvc.logger.Errorw("unexpected error in SLAM service", "error", err)
					}
					slamSvc.activeBackgroundWorkers.Done()
					return
				}
				goutils.PanicCapturingGo(func() {
					defer slamSvc.activeBackgroundWorkers.Done()
					switch slamSvc.slamLib.AlgoType {
					case slam.Dense:
						if _, err := slamSvc.getAndSaveDataDense(cancelCtx, cams); err != nil {
							slamSvc.logger.Warn(err)
						}
						if c != nil {
							c <- 1
						}
					case slam.Sparse:
						if _, err := slamSvc.getAndSaveDataSparse(cancelCtx, cams); err != nil {
							slamSvc.logger.Warn(err)
						}
						if c != nil {
							c <- 1
						}
					default:
						slamSvc.logger.Warnw("warning invalid algorithm specified", "algorithm", slamSvc.slamLib.AlgoType)
					}
				})
			}
		}
	})
}

// GetSLAMProcessConfig returns the process config for the SLAM process.
func (slamSvc *orbslamService) GetSLAMProcessConfig() pexec.ProcessConfig {
	var args []string

	args = append(args, "-sensors="+slamSvc.primarySensorName)
	args = append(args, "-config_param="+slamUtils.DictToString(slamSvc.configParams))
	args = append(args, "-data_rate_ms="+strconv.Itoa(slamSvc.dataRateMs))
	args = append(args, "-map_rate_sec="+strconv.Itoa(slamSvc.mapRateSec))
	args = append(args, "-data_dir="+slamSvc.dataDirectory)
	args = append(args, "-delete_processed_data="+strconv.FormatBool(slamSvc.deleteProcessedData))
	args = append(args, "-use_live_data="+strconv.FormatBool(slamSvc.useLiveData))
	args = append(args, "-port="+slamSvc.port)
	args = append(args, "--aix-auto-update")

	return pexec.ProcessConfig{
		ID:      "slam_" + slamSvc.slamLib.AlgoName,
		Name:    slam.SLAMLibraries[slamSvc.slamLib.AlgoName].BinaryLocation,
		Args:    args,
		Log:     true,
		OneShot: false,
	}
}

func (slamSvc *orbslamService) GetSLAMProcessBufferedLogReader() bufio.Reader {
	return slamSvc.slamProcessBufferedLogReader
}

// startSLAMProcess starts up the SLAM library process by calling the executable binary and giving it the necessary arguments.
func (slamSvc *orbslamService) StartSLAMProcess(ctx context.Context) error {
	ctx, span := trace.StartSpan(ctx, "slam::slamService::StartSLAMProcess")
	defer span.End()

	processConfig := slamSvc.GetSLAMProcessConfig()

	var logReader io.ReadCloser
	var logWriter io.WriteCloser
	var bufferedLogReader bufio.Reader
	if slamSvc.port == localhost0 || slamSvc.bufferSLAMProcessLogs {
		logReader, logWriter = io.Pipe()
		bufferedLogReader = *bufio.NewReader(logReader)
		processConfig.LogWriter = logWriter
	}

	_, err := slamSvc.slamProcess.AddProcessFromConfig(ctx, processConfig)
	if err != nil {
		return errors.Wrap(err, "problem adding slam process")
	}

	slamSvc.logger.Debug("starting slam process")

	if err = slamSvc.slamProcess.Start(ctx); err != nil {
		return errors.Wrap(err, "problem starting slam process")
	}

	if slamSvc.port == localhost0 {
		timeoutCtx, timeoutCancel := context.WithTimeout(ctx, parsePortMaxTimeoutSec*time.Second)
		defer timeoutCancel()

		if !slamSvc.bufferSLAMProcessLogs {
			defer func(logger golog.Logger) {
				if err := logReader.Close(); err != nil {
					logger.Debugw("Closing logReader returned an error", "error", err)
				}
			}(slamSvc.logger)
			defer func(logger golog.Logger) {
				if err := logWriter.Close(); err != nil {
					logger.Debugw("Closing logReader returned an error", "error", err)
				}
			}(slamSvc.logger)
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
				slamSvc.port = "localhost:" + strings.TrimRight(linePieces[1], "\n")
				break
			}
		}
	}

	if slamSvc.bufferSLAMProcessLogs {
		slamSvc.slamProcessLogReader = logReader
		slamSvc.slamProcessLogWriter = logWriter
		slamSvc.slamProcessBufferedLogReader = bufferedLogReader
	}

	return nil
}

// stopSLAMProcess uses the process manager to stop the created slam process from running.
func (slamSvc *orbslamService) StopSLAMProcess() error {
	if err := slamSvc.slamProcess.Stop(); err != nil {
		return errors.Wrap(err, "problem stopping slam process")
	}
	return nil
}

func (slamSvc *orbslamService) getPNGImage(ctx context.Context, cam camera.Camera) ([]byte, func(), error) {
	// We will hint that we want a PNG.
	// The Camera service server implementation in RDK respects this; others may not.
	readImgCtx := gostream.WithMIMETypeHint(ctx, rdkutils.WithLazyMIMEType(rdkutils.MimeTypePNG))
	img, release, err := camera.ReadImage(readImgCtx, cam)
	if err != nil {
		return nil, nil, err
	}
	if lazyImg, ok := img.(*rimage.LazyEncodedImage); ok {
		if lazyImg.MIMEType() != rdkutils.MimeTypePNG {
			return nil, nil, errors.Errorf("expected mime type %v, got %T", rdkutils.MimeTypePNG, img)
		}
		return lazyImg.RawData(), release, nil
	}

	if ycbcrImg, ok := img.(*image.YCbCr); ok {
		pngImage, err := rimage.EncodeImage(ctx, ycbcrImg, rdkutils.MimeTypePNG)
		if err != nil {
			return nil, nil, err
		}
		return pngImage, release, nil
	}

	return nil, nil, errors.Errorf("expected lazily encoded image or ycbcrImg, got %T", img)
}

// getAndSaveDataSparse implements the data extraction for sparse algos and saving to the directory path (data subfolder) specified in
// the config. It returns the full filepath for each file saved along with any error associated with the data creation or saving.
func (slamSvc *orbslamService) getAndSaveDataSparse(
	ctx context.Context,
	cams []camera.Camera,
) ([]string, error) {
	ctx, span := trace.StartSpan(ctx, "slam::orbslamService::getAndSaveDataSparse")
	defer span.End()

	switch slamSvc.subAlgo {
	case slam.Mono:
		if len(cams) != 1 {
			return nil, errors.Errorf("expected 1 camera for mono slam, found %v", len(cams))
		}

		image, release, err := slamSvc.getPNGImage(ctx, cams[0])
		if release != nil {
			defer release()
		}
		if err != nil {
			if err.Error() == opTimeoutErrorMessage {
				slamSvc.logger.Warnw("Skipping this scan due to error", "error", err)
				return nil, nil
			}
			return nil, err
		}
		filenames, err := createTimestampFilenames(slamSvc.dataDirectory, slamSvc.primarySensorName, ".png", slamSvc.subAlgo)
		if err != nil {
			return nil, err
		}

		filename := filenames[0]
		return []string{filename}, dataprocess.WriteBytesToFile(image, filename)
	case slam.Rgbd:
		if len(cams) != 2 {
			return nil, errors.Errorf("expected 2 cameras for Rgbd slam, found %v", len(cams))
		}

		images, releaseFuncs, err := slamSvc.getSimultaneousColorAndDepth(ctx, cams)
		for _, rFunc := range releaseFuncs {
			if rFunc != nil {
				defer rFunc()
			}
		}
		if err != nil {
			if err.Error() == opTimeoutErrorMessage {
				slamSvc.logger.Warnw("Skipping this scan due to error", "error", err)
				return nil, nil
			}
			return nil, err
		}

		filenames, err := createTimestampFilenames(slamSvc.dataDirectory, slamSvc.primarySensorName, ".png", slamSvc.subAlgo)
		if err != nil {
			return nil, err
		}
		for i, filename := range filenames {
			if err = dataprocess.WriteBytesToFile(images[i], filename); err != nil {
				return filenames, err
			}
		}
		return filenames, nil
	case slam.Dim2d:
		return nil, errors.Errorf("bad slamMode %v specified for this algorithm", slamSvc.slamMode)
	case slam.Dim3d:
		return nil, errors.New("Dim3d is not implemented")
	default:
		return nil, errors.Errorf("invalid subAlgo %v specified", slamSvc.subAlgo)
	}
}

// Gets the color image and depth image from the cameras as close to simultaneously as possible.
func (slamSvc *orbslamService) getSimultaneousColorAndDepth(
	ctx context.Context,
	cams []camera.Camera,
) ([2][]byte, [2]func(), error) {
	var wg sync.WaitGroup
	var images [2][]byte
	var releaseFuncs [2]func()
	var errs [2]error

	for i := 0; i < 2; i++ {
		slamSvc.activeBackgroundWorkers.Add(1)
		wg.Add(1)
		if err := ctx.Err(); err != nil {
			if !errors.Is(err, context.Canceled) {
				slamSvc.logger.Errorw("unexpected error in SLAM service", "error", err)
			}
			slamSvc.activeBackgroundWorkers.Done()
			return images, releaseFuncs, err
		}
		iLoop := i
		goutils.PanicCapturingGo(func() {
			defer slamSvc.activeBackgroundWorkers.Done()
			defer wg.Done()
			images[iLoop], releaseFuncs[iLoop], errs[iLoop] = slamSvc.getPNGImage(ctx, cams[iLoop])
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

// getAndSaveDataDense implements the data extraction for dense algos and saving to the directory path (data subfolder) specified in
// the config. It returns the full filepath for each file saved along with any error associated with the data creation or saving.
func (slamSvc *orbslamService) getAndSaveDataDense(ctx context.Context, cams []camera.Camera) (string, error) {
	ctx, span := trace.StartSpan(ctx, "slam::orbslamService::getAndSaveDataDense")
	defer span.End()

	if len(cams) != 1 {
		return "", errors.Errorf("expected 1 camera for this slam algorithm, found %v", len(cams))
	}

	pointcloud, err := cams[0].NextPointCloud(ctx)
	if err != nil {
		if err.Error() == opTimeoutErrorMessage {
			slamSvc.logger.Warnw("Skipping this scan due to error", "error", err)
			return "", nil
		}
		return "", err
	}

	var fileType string
	switch slamSvc.subAlgo {
	case slam.Dim2d:
		fileType = ".pcd"
	case slam.Rgbd, slam.Mono:
		return "", errors.Errorf("bad subAlgo %v specified for this algorithm", slamSvc.subAlgo)
	}
	filenames, err := createTimestampFilenames(slamSvc.dataDirectory, slamSvc.primarySensorName, fileType, slamSvc.subAlgo)
	if err != nil {
		return "", err
	}
	filename := filenames[0]
	return filename, dataprocess.WritePCDToFile(pointcloud, filename)
}

// Creates a file for camera data with the specified sensor name and timestamp written into the filename.
// For RGBD cameras, two filenames are created with the same timestamp in different directories.
func createTimestampFilenames(dataDirectory, primarySensorName, fileType string, subAlgo slam.Mode) ([]string, error) {
	timeStamp := time.Now()
	dataDir := filepath.Join(dataDirectory, "data")
	rbgDataDir := filepath.Join(dataDir, "rgb")
	depthDataDir := filepath.Join(dataDir, "depth")

	switch subAlgo {
	case slam.Dim2d:
		filename := dataprocess.CreateTimestampFilename(dataDir, primarySensorName, fileType, timeStamp)
		return []string{filename}, nil
	case slam.Mono:
		rgbFilename := dataprocess.CreateTimestampFilename(rbgDataDir, primarySensorName, fileType, timeStamp)
		return []string{rgbFilename}, nil
	case slam.Rgbd:
		rgbFilename := dataprocess.CreateTimestampFilename(rbgDataDir, primarySensorName, fileType, timeStamp)
		depthFilename := dataprocess.CreateTimestampFilename(depthDataDir, primarySensorName, fileType, timeStamp)
		return []string{rgbFilename, depthFilename}, nil
	case slam.Dim3d:
		return nil, errors.New("Dim3d is not implemented")
	default:
		return nil, errors.Errorf("Invalid sub algo: %v", subAlgo)
	}
}
