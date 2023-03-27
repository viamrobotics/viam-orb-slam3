// Package testhelper implements a slam service definition with additional exported functions for
// the purpose of testing
package testhelper

import (
	"context"
	"image"
	"net"
	"os"
	"strconv"
	"sync"
	"testing"

	"github.com/edaniels/golog"
	"github.com/edaniels/gostream"
	"github.com/pkg/errors"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/config"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/rimage"
	"go.viam.com/rdk/rimage/transform"
	"go.viam.com/rdk/services/slam"
	"go.viam.com/rdk/testutils/inject"
	rdkutils "go.viam.com/rdk/utils"
	slamConfig "go.viam.com/slam/config"
	slamTesthelper "go.viam.com/slam/testhelper"
	"go.viam.com/test"
	"go.viam.com/utils/artifact"
	"google.golang.org/grpc"

	viamorbslam3 "github.com/viamrobotics/viam-orb-slam3"

	"bufio"
	"bytes"
	"go.viam.com/rdk/registry"
	"go.viam.com/utils/pexec"
	"sync/atomic"
)

var (
	IntCameraMutex             sync.Mutex
	IntCameraReleaseImagesChan = make(chan int, 2)
	IntWebcamReleaseImageChan  = make(chan int, 1)
	IntSynchronizeCamerasChan  = make(chan int)
)

func SetupDeps(attr *slamConfig.AttrConfig) registry.Dependencies {
	deps := make(registry.Dependencies)
	var projA transform.Projector
	intrinsicsA := &transform.PinholeCameraIntrinsics{ // not the real camera parameters -- fake for test
		Width:  1280,
		Height: 720,
		Fx:     200,
		Fy:     200,
		Ppx:    640,
		Ppy:    360,
	}
	distortionsA := &transform.BrownConrady{RadialK1: 0.001, RadialK2: 0.00004}
	projA = intrinsicsA

	var projRealSense transform.Projector
	intrinsicsRealSense := &transform.PinholeCameraIntrinsics{
		Width:  1280,
		Height: 720,
		Fx:     900.538,
		Fy:     900.818,
		Ppx:    648.934,
		Ppy:    367.736,
	}
	distortionsRealSense := &transform.BrownConrady{
		RadialK1:     0.158701,
		RadialK2:     -0.485405,
		RadialK3:     0.435342,
		TangentialP1: -0.00143327,
		TangentialP2: -0.000705919,
	}
	projRealSense = intrinsicsRealSense

	var projWebcam transform.Projector
	intrinsicsWebcam := &transform.PinholeCameraIntrinsics{
		Width:  640,
		Height: 480,
		Fx:     939.2693584627577,
		Fy:     940.2928257873841,
		Ppx:    320.6075282958033,
		Ppy:    239.14408757087756,
	}
	distortionsWebcam := &transform.BrownConrady{
		RadialK1:     0.046535971648456166,
		RadialK2:     0.8002516496932317,
		RadialK3:     -5.408034254951954,
		TangentialP1: -8.996658362365533e-06,
		TangentialP2: -0.002828504714921335,
	}
	projWebcam = intrinsicsWebcam

	for _, sensor := range attr.Sensors {
		cam := &inject.Camera{}
		switch sensor {
		case "invalid_sensor_type":
			cam.StreamFunc = func(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
				return nil, errors.New("this device does not stream images")
			}
			cam.ProjectorFunc = func(ctx context.Context) (transform.Projector, error) {
				return nil, transform.NewNoIntrinsicsError("")
			}
			deps[camera.Named(sensor)] = cam
		case "good_camera":
			cam = GetGoodOrMissingDistortionParamsCamera(projA)
			cam.PropertiesFunc = func(ctx context.Context) (camera.Properties, error) {
				return camera.Properties{IntrinsicParams: intrinsicsA, DistortionParams: distortionsA}, nil
			}
			deps[camera.Named(sensor)] = cam
		case "missing_distortion_parameters_camera":
			cam = GetGoodOrMissingDistortionParamsCamera(projA)
			cam.PropertiesFunc = func(ctx context.Context) (camera.Properties, error) {
				return camera.Properties{IntrinsicParams: intrinsicsA, DistortionParams: nil}, nil
			}
			deps[camera.Named(sensor)] = cam
		case "missing_camera_properties":
			cam.StreamFunc = func(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
				return gostream.NewEmbeddedVideoStreamFromReader(
					gostream.VideoReaderFunc(func(ctx context.Context) (image.Image, func(), error) {
						return image.NewNRGBA(image.Rect(0, 0, 1024, 1024)), nil, nil
					}),
				), nil
			}
			cam.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
				return nil, errors.New("camera not lidar")
			}
			cam.ProjectorFunc = func(ctx context.Context) (transform.Projector, error) {
				return projA, nil
			}
			cam.PropertiesFunc = func(ctx context.Context) (camera.Properties, error) {
				return camera.Properties{}, errors.New("somehow couldn't get properties")
			}
			deps[camera.Named(sensor)] = cam
		case "good_color_camera":
			cam.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
				return nil, errors.New("camera not lidar")
			}
			cam.ProjectorFunc = func(ctx context.Context) (transform.Projector, error) {
				return projA, nil
			}
			cam.PropertiesFunc = func(ctx context.Context) (camera.Properties, error) {
				return camera.Properties{IntrinsicParams: intrinsicsA, DistortionParams: distortionsA}, nil
			}
			cam.StreamFunc = func(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
				imgBytes, err := os.ReadFile(artifact.MustPath("rimage/board1.png"))
				if err != nil {
					return nil, err
				}
				lazy := rimage.NewLazyEncodedImage(imgBytes, rdkutils.MimeTypePNG)
				return gostream.NewEmbeddedVideoStreamFromReader(
					gostream.VideoReaderFunc(func(ctx context.Context) (image.Image, func(), error) {
						return lazy, func() {}, nil
					}),
				), nil
			}
			deps[camera.Named(sensor)] = cam
		case "good_depth_camera":
			cam.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
				return nil, errors.New("camera not lidar")
			}
			cam.ProjectorFunc = func(ctx context.Context) (transform.Projector, error) {
				return nil, transform.NewNoIntrinsicsError("")
			}
			cam.PropertiesFunc = func(ctx context.Context) (camera.Properties, error) {
				return camera.Properties{}, nil
			}
			cam.StreamFunc = func(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
				imgBytes, err := os.ReadFile(artifact.MustPath("rimage/board1_gray.png"))
				if err != nil {
					return nil, err
				}
				lazy := rimage.NewLazyEncodedImage(imgBytes, rdkutils.MimeTypePNG)
				return gostream.NewEmbeddedVideoStreamFromReader(
					gostream.VideoReaderFunc(func(ctx context.Context) (image.Image, func(), error) {
						return lazy, func() {}, nil
					}),
				), nil
			}
			deps[camera.Named(sensor)] = cam
		case "bad_camera_no_stream":
			cam.StreamFunc = func(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
				return nil, errors.New("bad_camera_no_stream")
			}
			cam.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
				return nil, errors.New("camera not lidar")
			}
			cam.ProjectorFunc = func(ctx context.Context) (transform.Projector, error) {
				return projA, nil
			}
			cam.PropertiesFunc = func(ctx context.Context) (camera.Properties, error) {
				return camera.Properties{IntrinsicParams: intrinsicsA, DistortionParams: distortionsA}, nil
			}
			deps[camera.Named(sensor)] = cam
		case "bad_camera_intrinsics":
			cam.StreamFunc = func(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
				return gostream.NewEmbeddedVideoStreamFromReader(
					gostream.VideoReaderFunc(func(ctx context.Context) (image.Image, func(), error) {
						return image.NewNRGBA(image.Rect(0, 0, 1024, 1024)), nil, nil
					}),
				), nil
			}
			cam.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
				return nil, errors.New("camera not lidar")
			}
			cam.ProjectorFunc = func(ctx context.Context) (transform.Projector, error) {
				return &transform.PinholeCameraIntrinsics{}, nil
			}
			cam.PropertiesFunc = func(ctx context.Context) (camera.Properties, error) {
				return camera.Properties{
					IntrinsicParams:  &transform.PinholeCameraIntrinsics{},
					DistortionParams: &transform.BrownConrady{},
				}, nil
			}
			deps[camera.Named(sensor)] = cam
		case "orbslam_int_color_camera":
			cam.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
				return nil, errors.New("camera not lidar")
			}
			cam.ProjectorFunc = func(ctx context.Context) (transform.Projector, error) {
				return projRealSense, nil
			}
			cam.PropertiesFunc = func(ctx context.Context) (camera.Properties, error) {
				return camera.Properties{IntrinsicParams: intrinsicsRealSense, DistortionParams: distortionsRealSense}, nil
			}
			var index uint64
			cam.StreamFunc = func(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
				defer func() {
					IntSynchronizeCamerasChan <- 1
				}()
				// Ensure the StreamFunc functions for orbslam_int_color_camera and orbslam_int_depth_camera run under
				// the lock so that they release images in the same call to getSimultaneousColorAndDepth().
				IntCameraMutex.Lock()
				select {
				case <-IntCameraReleaseImagesChan:
					i := atomic.AddUint64(&index, 1) - 1
					if i >= uint64(GetNumOrbslamImages(viamorbslam3.Rgbd)) {
						return nil, errors.New("No more orbslam color images")
					}
					imgBytes, err := os.ReadFile(artifact.MustPath("slam/mock_camera_short/rgb/" + strconv.FormatUint(i, 10) + ".png"))
					if err != nil {
						return nil, err
					}
					lazy := rimage.NewLazyEncodedImage(imgBytes, rdkutils.MimeTypePNG)
					return gostream.NewEmbeddedVideoStreamFromReader(
						gostream.VideoReaderFunc(func(ctx context.Context) (image.Image, func(), error) {
							return lazy, func() {}, nil
						}),
					), nil
				default:
					return nil, errors.Errorf("Color camera not ready to return image %v", index)
				}
			}
			deps[camera.Named(sensor)] = cam
		case "orbslam_int_depth_camera":
			cam.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
				return nil, errors.New("camera not lidar")
			}
			cam.ProjectorFunc = func(ctx context.Context) (transform.Projector, error) {
				return nil, transform.NewNoIntrinsicsError("")
			}
			cam.PropertiesFunc = func(ctx context.Context) (camera.Properties, error) {
				return camera.Properties{}, nil
			}
			var index uint64
			cam.StreamFunc = func(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
				defer func() {
					IntCameraMutex.Unlock()
				}()
				// Ensure StreamFunc for orbslam_int_color_camera runs first, so that we lock orbslamIntCameraMutex before
				// unlocking it
				<-IntSynchronizeCamerasChan
				select {
				case <-IntCameraReleaseImagesChan:
					i := atomic.AddUint64(&index, 1) - 1
					if i >= uint64(GetNumOrbslamImages(viamorbslam3.Rgbd)) {
						return nil, errors.New("No more orbslam depth images")
					}
					imgBytes, err := os.ReadFile(artifact.MustPath("slam/mock_camera_short/depth/" + strconv.FormatUint(i, 10) + ".png"))
					if err != nil {
						return nil, err
					}
					lazy := rimage.NewLazyEncodedImage(imgBytes, rdkutils.MimeTypePNG)
					return gostream.NewEmbeddedVideoStreamFromReader(
						gostream.VideoReaderFunc(func(ctx context.Context) (image.Image, func(), error) {
							return lazy, func() {}, nil
						}),
					), nil
				default:
					return nil, errors.Errorf("Depth camera not ready to return image %v", index)
				}
			}
			deps[camera.Named(sensor)] = cam
		case "orbslam_int_webcam":
			cam.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
				return nil, errors.New("camera not lidar")
			}
			cam.ProjectorFunc = func(ctx context.Context) (transform.Projector, error) {
				return projWebcam, nil
			}
			cam.PropertiesFunc = func(ctx context.Context) (camera.Properties, error) {
				return camera.Properties{IntrinsicParams: intrinsicsWebcam, DistortionParams: distortionsWebcam}, nil
			}
			var index uint64
			cam.StreamFunc = func(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
				select {
				case <-IntWebcamReleaseImageChan:
					i := atomic.AddUint64(&index, 1) - 1
					if i >= uint64(GetNumOrbslamImages(viamorbslam3.Mono)) {
						return nil, errors.New("No more orbslam webcam images")
					}
					imgBytes, err := os.ReadFile(artifact.MustPath("slam/mock_mono_camera/rgb/" + strconv.FormatUint(i, 10) + ".png"))
					if err != nil {
						return nil, err
					}
					img, _, err := image.Decode(bytes.NewReader(imgBytes))
					if err != nil {
						return nil, err
					}
					var ycbcrImg image.YCbCr
					rimage.ImageToYCbCrForTesting(&ycbcrImg, img)
					return gostream.NewEmbeddedVideoStreamFromReader(
						gostream.VideoReaderFunc(func(ctx context.Context) (image.Image, func(), error) {
							return &ycbcrImg, func() {}, nil
						}),
					), nil
				default:
					return nil, errors.Errorf("Webcam not ready to return image %v", index)
				}
			}
			deps[camera.Named(sensor)] = cam
		case "gibberish":
			return deps
		default:
			continue
		}
	}
	return deps
}

func GetGoodOrMissingDistortionParamsCamera(projA transform.Projector) *inject.Camera {
	cam := &inject.Camera{}
	cam.StreamFunc = func(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
		return gostream.NewEmbeddedVideoStreamFromReader(
			gostream.VideoReaderFunc(func(ctx context.Context) (image.Image, func(), error) {
				return image.NewNRGBA(image.Rect(0, 0, 1024, 1024)), nil, nil
			}),
		), nil
	}
	cam.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
		return nil, errors.New("camera not lidar")
	}
	cam.ProjectorFunc = func(ctx context.Context) (transform.Projector, error) {
		return projA, nil
	}
	return cam
}

func GetNumOrbslamImages(mode viamorbslam3.SubAlgo) int {
	switch mode {
	case viamorbslam3.Mono:
		return 15
	case viamorbslam3.Rgbd:
		return 29
	default:
		return 0
	}
}

func CloseOutSLAMService(t *testing.T, name string) {
	t.Helper()

	if name != "" {
		err := slamTesthelper.ResetFolder(name)
		test.That(t, err, test.ShouldBeNil)
	}
}

func SetupTestGRPCServer(tb testing.TB) (*grpc.Server, int) {
	listener, err := net.Listen("tcp", ":0")
	test.That(tb, err, test.ShouldBeNil)
	grpcServer := grpc.NewServer()
	go grpcServer.Serve(listener)

	return grpcServer, listener.Addr().(*net.TCPAddr).Port
}

func CreateSLAMService(
	t *testing.T,
	attrCfg *slamConfig.AttrConfig,
	logger golog.Logger,
	// TODO(RSDK-2026) will be fixed once integration tests use this option in the next few PRs
	//nolint:unparam
	bufferSLAMProcessLogs bool,
	success bool,
	executableName string,
) (slam.Service, error) {
	t.Helper()

	ctx := context.Background()
	cfgService := config.Service{Name: "test", Type: "slam", Model: viamorbslam3.Model}
	cfgService.ConvertedAttributes = attrCfg

	deps := SetupDeps(attrCfg)

	sensorDeps, err := attrCfg.Validate("path")
	if err != nil {
		return nil, err
	}
	test.That(t, sensorDeps, test.ShouldResemble, attrCfg.Sensors)

	viamorbslam3.SetCameraValidationMaxTimeoutSecForTesting(1)
	viamorbslam3.SetDialMaxTimeoutSecForTesting(1)

	svc, err := viamorbslam3.New(ctx, deps, cfgService, logger, bufferSLAMProcessLogs, executableName)

	if success {
		if err != nil {
			return nil, err
		}
		test.That(t, svc, test.ShouldNotBeNil)
		return svc, nil
	}

	test.That(t, svc, test.ShouldBeNil)
	return nil, err
}

// Service in the internal package includes additional exported functions relating to the data and
// slam processes in the slam service. These functions are not exported to the user. This resolves
// a circular import caused by the inject package.
type Service interface {
	StartDataProcess(cancelCtx context.Context, cam []camera.Camera, c chan int)
	StartSLAMProcess(ctx context.Context) error
	StopSLAMProcess() error
	Close() error
	GetSLAMProcessConfig() pexec.ProcessConfig
	GetSLAMProcessBufferedLogReader() bufio.Reader
}
