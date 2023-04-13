// Package viamorbslam3_test tests the functions that required injected components (such as robot and camera)
// in order to be run. It utilizes the internal package located in slam_test_helper.go to access
// certain exported functions which we do not want to make available to the user.
package viamorbslam3_test

import (
	"bytes"
	"context"
	"fmt"
	"image"
	"net"
	"os"
	"strconv"
	"sync"
	"sync/atomic"
	"testing"

	"github.com/edaniels/golog"
	"github.com/edaniels/gostream"
	"github.com/pkg/errors"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/config"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/registry"
	"go.viam.com/rdk/rimage"
	"go.viam.com/rdk/rimage/transform"
	"go.viam.com/rdk/services/slam"
	"go.viam.com/rdk/testutils/inject"
	rdkutils "go.viam.com/rdk/utils"
	slamConfig "go.viam.com/slam/config"
	slamTesthelper "go.viam.com/slam/testhelper"
	"go.viam.com/test"
	"go.viam.com/utils"
	"go.viam.com/utils/artifact"
	"google.golang.org/grpc"

	viamorbslam3 "github.com/viamrobotics/viam-orb-slam3"
	"github.com/viamrobotics/viam-orb-slam3/internal/testhelper"
)

const (
	validDataRateMS    = 200
	dataBufferSize     = 4
	testExecutableName = "true" // the program "true", not the boolean value
)

var (
	orbslamIntCameraMutex             sync.Mutex
	orbslamIntCameraReleaseImagesChan = make(chan int, 2)
	orbslamIntWebcamReleaseImageChan  = make(chan int, 1)
	orbslamIntSynchronizeCamerasChan  = make(chan int)
	validMapRate                      = 200
	_true                             = true
	_false                            = false
)

func getNumOrbslamImages(mode viamorbslam3.SubAlgo) int {
	switch mode {
	case viamorbslam3.Mono:
		return 15
	case viamorbslam3.Rgbd:
		return 29
	default:
		return 0
	}
}

func closeOutSLAMService(t *testing.T, name string) {
	t.Helper()

	if name != "" {
		err := slamTesthelper.ResetFolder(name)
		test.That(t, err, test.ShouldBeNil)
	}
}

func setupTestGRPCServer(tb testing.TB) (*grpc.Server, int) {
	listener, err := net.Listen("tcp", ":0")
	test.That(tb, err, test.ShouldBeNil)
	grpcServer := grpc.NewServer()
	go grpcServer.Serve(listener)

	return grpcServer, listener.Addr().(*net.TCPAddr).Port
}

func getGoodOrMissingDistortionParamsCamera(projA transform.Projector) *inject.Camera {
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

func setupDeps(attr *slamConfig.AttrConfig) registry.Dependencies {
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
			cam = getGoodOrMissingDistortionParamsCamera(projA)
			cam.PropertiesFunc = func(ctx context.Context) (camera.Properties, error) {
				return camera.Properties{IntrinsicParams: intrinsicsA, DistortionParams: distortionsA}, nil
			}
			deps[camera.Named(sensor)] = cam
		case "missing_distortion_parameters_camera":
			cam = getGoodOrMissingDistortionParamsCamera(projA)
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
					orbslamIntSynchronizeCamerasChan <- 1
				}()
				// Ensure the StreamFunc functions for orbslam_int_color_camera and orbslam_int_depth_camera run under
				// the lock so that they release images in the same call to getSimultaneousColorAndDepth().
				orbslamIntCameraMutex.Lock()
				select {
				case <-orbslamIntCameraReleaseImagesChan:
					i := atomic.AddUint64(&index, 1) - 1
					if i >= uint64(getNumOrbslamImages(viamorbslam3.Rgbd)) {
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
					orbslamIntCameraMutex.Unlock()
				}()
				// Ensure StreamFunc for orbslam_int_color_camera runs first, so that we lock orbslamIntCameraMutex before
				// unlocking it
				<-orbslamIntSynchronizeCamerasChan
				select {
				case <-orbslamIntCameraReleaseImagesChan:
					i := atomic.AddUint64(&index, 1) - 1
					if i >= uint64(getNumOrbslamImages(viamorbslam3.Rgbd)) {
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
				case <-orbslamIntWebcamReleaseImageChan:
					i := atomic.AddUint64(&index, 1) - 1
					if i >= uint64(getNumOrbslamImages(viamorbslam3.Mono)) {
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

func createSLAMService(
	t *testing.T,
	attrCfg *slamConfig.AttrConfig,
	logger golog.Logger,
	bufferSLAMProcessLogs bool,
	success bool,
	executableName string,
) (slam.Service, error) {
	t.Helper()

	ctx := context.Background()
	cfgService := config.Service{Name: "test", Type: "slam", Model: viamorbslam3.Model}
	cfgService.ConvertedAttributes = attrCfg

	deps := setupDeps(attrCfg)

	sensorDeps, err := attrCfg.Validate("path")
	if err != nil {
		return nil, err
	}
	test.That(t, sensorDeps, test.ShouldResemble, attrCfg.Sensors)

	viamorbslam3.SetCameraValidationMaxTimeoutSecForTesting(1)
	viamorbslam3.SetDialMaxTimeoutSecForTesting(2)

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

func TestGeneralNew(t *testing.T) {
	logger := golog.NewTestLogger(t)
	name, err := slamTesthelper.CreateTempFolderArchitecture(logger)
	test.That(t, err, test.ShouldBeNil)

	t.Run("New slam service with no camera", func(t *testing.T) {
		grpcServer, port := setupTestGRPCServer(t)
		test.That(t, err, test.ShouldBeNil)
		attrCfg := &slamConfig.AttrConfig{
			Sensors:       []string{},
			ConfigParams:  map[string]string{"mode": "mono"},
			DataDirectory: name,
			Port:          "localhost:" + strconv.Itoa(port),
			UseLiveData:   &_false,
		}

		// Create slam service

		test.That(t, err, test.ShouldBeNil)
		svc, err := createSLAMService(t, attrCfg, logger, false, true, testExecutableName)
		test.That(t, err, test.ShouldBeNil)

		grpcServer.Stop()
		test.That(t, utils.TryClose(context.Background(), svc), test.ShouldBeNil)
	})

	t.Run("New slam service with bad camera", func(t *testing.T) {
		attrCfg := &slamConfig.AttrConfig{
			Sensors:       []string{"gibberish"},
			ConfigParams:  map[string]string{"mode": "mono"},
			DataDirectory: name,
			DataRateMsec:  validDataRateMS,
			UseLiveData:   &_true,
		}

		// Create slam service
		_, err := createSLAMService(t, attrCfg, logger, false, false, testExecutableName)
		test.That(t, err, test.ShouldBeError,
			errors.New("configuring camera error: error getting camera gibberish for slam service: "+
				"\"gibberish\" missing from dependencies"))
	})

	closeOutSLAMService(t, name)
}

func TestORBSLAMNew(t *testing.T) {
	logger := golog.NewTestLogger(t)
	name, err := slamTesthelper.CreateTempFolderArchitecture(logger)
	test.That(t, err, test.ShouldBeNil)

	t.Run("New orbslamv3 service with good camera in slam mode rgbd", func(t *testing.T) {
		grpcServer, port := setupTestGRPCServer(t)
		attrCfg := &slamConfig.AttrConfig{
			Sensors:       []string{"good_color_camera", "good_depth_camera"},
			ConfigParams:  map[string]string{"mode": "rgbd"},
			DataDirectory: name,
			DataRateMsec:  validDataRateMS,
			Port:          "localhost:" + strconv.Itoa(port),
			UseLiveData:   &_true,
		}

		// Create slam service
		svc, err := createSLAMService(t, attrCfg, logger, false, true, testExecutableName)
		test.That(t, err, test.ShouldBeNil)

		grpcServer.Stop()
		test.That(t, utils.TryClose(context.Background(), svc), test.ShouldBeNil)
	})

	t.Run("New orbslamv3 service in slam mode rgbd that errors due to a single camera", func(t *testing.T) {
		attrCfg := &slamConfig.AttrConfig{
			Sensors:       []string{"good_color_camera"},
			ConfigParams:  map[string]string{"mode": "rgbd"},
			DataDirectory: name,
			DataRateMsec:  validDataRateMS,
			UseLiveData:   &_true,
		}

		// Create slam service
		_, err = createSLAMService(t, attrCfg, logger, false, false, testExecutableName)
		test.That(t, err.Error(), test.ShouldContainSubstring,
			errors.Errorf("expected 2 cameras for Rgbd slam, found %v", len(attrCfg.Sensors)).Error())
	})

	t.Run("New orbslamv3 service that errors due to missing distortion_parameters not being provided in config", func(t *testing.T) {
		grpcServer, port := setupTestGRPCServer(t)
		attrCfg := &slamConfig.AttrConfig{
			Sensors:       []string{"missing_distortion_parameters_camera"},
			ConfigParams:  map[string]string{"mode": "mono"},
			DataDirectory: name,
			DataRateMsec:  validDataRateMS,
			Port:          "localhost:" + strconv.Itoa(port),
			UseLiveData:   &_true,
		}

		// Create slam service
		svc, err := createSLAMService(t, attrCfg, logger, false, true, testExecutableName)
		expectedError := errors.New("configuring camera error: error getting distortion_parameters for slam " +
			"service, only BrownConrady distortion parameters are supported").Error()
		test.That(t, err.Error(), test.ShouldContainSubstring, expectedError)

		grpcServer.Stop()
		test.That(t, utils.TryClose(context.Background(), svc), test.ShouldBeNil)
	})

	t.Run("New orbslamv3 service that errors due to not being able to get camera properties", func(t *testing.T) {
		grpcServer, port := setupTestGRPCServer(t)
		attrCfg := &slamConfig.AttrConfig{
			Sensors:       []string{"missing_camera_properties"},
			ConfigParams:  map[string]string{"mode": "mono"},
			DataDirectory: name,
			DataRateMsec:  validDataRateMS,
			Port:          "localhost:" + strconv.Itoa(port),
			UseLiveData:   &_true,
		}

		// Create slam service
		logger := golog.NewTestLogger(t)
		svc, err := createSLAMService(t, attrCfg, logger, false, true, testExecutableName)
		expectedError := errors.New("configuring camera error: error getting camera properties for slam " +
			"service: somehow couldn't get properties").Error()
		test.That(t, err.Error(), test.ShouldContainSubstring, expectedError)

		grpcServer.Stop()
		test.That(t, utils.TryClose(context.Background(), svc), test.ShouldBeNil)
	})

	t.Run("New orbslamv3 service in slam mode rgbd that errors due cameras in the wrong order", func(t *testing.T) {
		attrCfg := &slamConfig.AttrConfig{
			Sensors:       []string{"good_depth_camera", "good_color_camera"},
			ConfigParams:  map[string]string{"mode": "rgbd"},
			DataDirectory: name,
			DataRateMsec:  validDataRateMS,
			UseLiveData:   &_true,
		}

		// Create slam service
		_, err = createSLAMService(t, attrCfg, logger, false, false, testExecutableName)
		test.That(t, err.Error(), test.ShouldContainSubstring,
			errors.New("Unable to get camera features for first camera, make sure the color camera is listed first").Error())
	})

	t.Run("New orbslamv3 service with good camera in slam mode mono", func(t *testing.T) {
		grpcServer, port := setupTestGRPCServer(t)
		attrCfg := &slamConfig.AttrConfig{
			Sensors:       []string{"good_color_camera"},
			ConfigParams:  map[string]string{"mode": "mono"},
			DataDirectory: name,
			DataRateMsec:  validDataRateMS,
			Port:          "localhost:" + strconv.Itoa(port),
			UseLiveData:   &_true,
		}

		// Create slam service
		svc, err := createSLAMService(t, attrCfg, logger, false, true, testExecutableName)
		test.That(t, err, test.ShouldBeNil)

		grpcServer.Stop()
		test.That(t, utils.TryClose(context.Background(), svc), test.ShouldBeNil)
	})

	t.Run("New orbslamv3 service with camera that errors during call to Next", func(t *testing.T) {
		attrCfg := &slamConfig.AttrConfig{
			Sensors:       []string{"bad_camera_no_stream"},
			ConfigParams:  map[string]string{"mode": "mono"},
			DataDirectory: name,
			DataRateMsec:  validDataRateMS,
			UseLiveData:   &_true,
		}

		// Create slam service
		_, err := createSLAMService(t, attrCfg, logger, false, false, testExecutableName)
		test.That(t, err, test.ShouldBeError,
			errors.Errorf("runtime slam service error: "+
				"error getting data in desired mode: %v", attrCfg.Sensors[0]))
	})
	t.Run("New orbslamv3 service with camera that errors from bad intrinsics", func(t *testing.T) {
		attrCfg := &slamConfig.AttrConfig{
			Sensors:       []string{"bad_camera_intrinsics"},
			ConfigParams:  map[string]string{"mode": "mono"},
			DataDirectory: name,
			DataRateMsec:  validDataRateMS,
			UseLiveData:   &_true,
		}

		// Create slam service
		_, err := createSLAMService(t, attrCfg, logger, false, false, testExecutableName)

		test.That(t, err.Error(), test.ShouldContainSubstring,
			transform.NewNoIntrinsicsError(fmt.Sprintf("Invalid size (%#v, %#v)", 0, 0)).Error())
	})
	t.Run("New orbslamv3 service with invalid sensor without Next implementation", func(t *testing.T) {
		attrCfg := &slamConfig.AttrConfig{
			Sensors:       []string{"invalid_sensor_type"},
			ConfigParams:  map[string]string{"mode": "mono"},
			DataDirectory: name,
			DataRateMsec:  validDataRateMS,
			UseLiveData:   &_true,
		}

		// Create slam service
		_, err := createSLAMService(t, attrCfg, logger, false, false, testExecutableName)
		test.That(t, err.Error(), test.ShouldContainSubstring,
			"configuring camera error:")
	})
	closeOutSLAMService(t, name)
}

func TestORBSLAMDataProcess(t *testing.T) {
	logger, obs := golog.NewObservedTestLogger(t)
	name, err := slamTesthelper.CreateTempFolderArchitecture(logger)
	test.That(t, err, test.ShouldBeNil)

	grpcServer, port := setupTestGRPCServer(t)
	attrCfg := &slamConfig.AttrConfig{
		Sensors:       []string{"good_color_camera"},
		ConfigParams:  map[string]string{"mode": "mono"},
		DataDirectory: name,
		DataRateMsec:  validDataRateMS,
		Port:          "localhost:" + strconv.Itoa(port),
		UseLiveData:   &_true,
	}

	// Create slam service
	svc, err := createSLAMService(t, attrCfg, logger, false, true, testExecutableName)
	test.That(t, err, test.ShouldBeNil)

	grpcServer.Stop()
	test.That(t, utils.TryClose(context.Background(), svc), test.ShouldBeNil)

	orbSvc := svc.(testhelper.Service)

	t.Run("ORBSLAM3 Data Process with camera in slam mode mono", func(t *testing.T) {
		goodCam := &inject.Camera{}
		goodCam.StreamFunc = func(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
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

		cams := []camera.Camera{goodCam}

		cancelCtx, cancelFunc := context.WithCancel(context.Background())

		c := make(chan int, 100)
		orbSvc.StartDataProcess(cancelCtx, cams, c)

		<-c
		cancelFunc()
		files, err := os.ReadDir(name + "/data/rgb/")
		test.That(t, len(files), test.ShouldBeGreaterThanOrEqualTo, 1)
		test.That(t, err, test.ShouldBeNil)
	})

	t.Run("ORBSLAM3 Data Process with camera that errors during call to Next", func(t *testing.T) {
		badCam := &inject.Camera{}
		badCam.StreamFunc = func(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
			return nil, errors.New("bad_camera")
		}
		cams := []camera.Camera{badCam}

		cancelCtx, cancelFunc := context.WithCancel(context.Background())
		c := make(chan int, 100)
		orbSvc.StartDataProcess(cancelCtx, cams, c)

		<-c
		obsAll := obs.All()
		latestLoggedEntry := obsAll[len(obsAll)-1]
		cancelFunc()
		test.That(t, fmt.Sprint(latestLoggedEntry), test.ShouldContainSubstring, "bad_camera")
	})

	test.That(t, utils.TryClose(context.Background(), svc), test.ShouldBeNil)

	closeOutSLAMService(t, name)
}

func TestEndpointFailures(t *testing.T) {
	logger := golog.NewTestLogger(t)
	name, err := slamTesthelper.CreateTempFolderArchitecture(logger)
	test.That(t, err, test.ShouldBeNil)

	grpcServer, port := setupTestGRPCServer(t)
	attrCfg := &slamConfig.AttrConfig{
		Sensors:       []string{"good_color_camera"},
		ConfigParams:  map[string]string{"mode": "mono", "test_param": "viam"},
		DataDirectory: name,
		MapRateSec:    &validMapRate,
		DataRateMsec:  validDataRateMS,
		Port:          "localhost:" + strconv.Itoa(port),
		UseLiveData:   &_true,
	}

	// Create slam service
	svc, err := createSLAMService(t, attrCfg, logger, false, true, testExecutableName)
	test.That(t, err, test.ShouldBeNil)

	pNew, frame, err := svc.GetPosition(context.Background())
	test.That(t, pNew, test.ShouldBeNil)
	test.That(t, frame, test.ShouldBeEmpty)
	test.That(t, fmt.Sprint(err), test.ShouldContainSubstring, "error getting SLAM position")

	callbackPointCloud, err := svc.GetPointCloudMap(context.Background())
	test.That(t, err, test.ShouldBeNil)
	test.That(t, callbackPointCloud, test.ShouldNotBeNil)
	chunkPCD, err := callbackPointCloud()
	test.That(t, err.Error(), test.ShouldContainSubstring, "error receiving pointcloud chunk")
	test.That(t, chunkPCD, test.ShouldBeNil)

	callbackInternalState, err := svc.GetInternalState(context.Background())
	test.That(t, err, test.ShouldBeNil)
	test.That(t, callbackInternalState, test.ShouldNotBeNil)
	chunkInternalState, err := callbackInternalState()
	test.That(t, err.Error(), test.ShouldContainSubstring, "error receiving internal state chunk")
	test.That(t, chunkInternalState, test.ShouldBeNil)

	grpcServer.Stop()
	test.That(t, utils.TryClose(context.Background(), svc), test.ShouldBeNil)

	closeOutSLAMService(t, name)
}

func TestSLAMProcessSuccess(t *testing.T) {
	logger := golog.NewTestLogger(t)
	name, err := slamTesthelper.CreateTempFolderArchitecture(logger)
	test.That(t, err, test.ShouldBeNil)

	t.Run("Test online SLAM process with default parameters", func(t *testing.T) {
		grpcServer, port := setupTestGRPCServer(t)
		attrCfg := &slamConfig.AttrConfig{
			Sensors:       []string{"good_color_camera", "good_depth_camera"},
			ConfigParams:  map[string]string{"mode": "rgbd", "test_param": "viam"},
			DataDirectory: name,
			Port:          "localhost:" + strconv.Itoa(port),
			UseLiveData:   &_true,
		}

		// Create slam service
		svc, err := createSLAMService(t, attrCfg, logger, false, true, testExecutableName)
		test.That(t, err, test.ShouldBeNil)

		orbSvc := svc.(testhelper.Service)
		processCfg := orbSvc.GetSLAMProcessConfig()
		cmd := append([]string{processCfg.Name}, processCfg.Args...)

		cmdResult := [][]string{
			{testExecutableName},
			{"-sensors=good_color_camera"},
			{"-config_param={test_param=viam,mode=rgbd}", "-config_param={mode=rgbd,test_param=viam}"},
			{"-data_rate_ms=200"},
			{"-map_rate_sec=60"},
			{"-data_dir=" + name},
			{"-delete_processed_data=true"},
			{"-use_live_data=true"},
			{"-port=localhost:" + strconv.Itoa(port)},
			{"--aix-auto-update"},
		}

		for i, s := range cmd {
			t.Run(fmt.Sprintf("Test command argument %v at index %v", s, i), func(t *testing.T) {
				test.That(t, s, test.ShouldBeIn, cmdResult[i])
			})
		}

		grpcServer.Stop()
		test.That(t, utils.TryClose(context.Background(), svc), test.ShouldBeNil)
	})

	t.Run("Test offline SLAM process with default parameters", func(t *testing.T) {
		grpcServer, port := setupTestGRPCServer(t)
		attrCfg := &slamConfig.AttrConfig{
			Sensors:       []string{},
			ConfigParams:  map[string]string{"mode": "mono", "test_param": "viam"},
			DataDirectory: name,
			Port:          "localhost:" + strconv.Itoa(port),
			UseLiveData:   &_false,
		}

		// Create slam service
		svc, err := createSLAMService(t, attrCfg, logger, false, true, testExecutableName)
		test.That(t, err, test.ShouldBeNil)

		orbSvc := svc.(testhelper.Service)
		processCfg := orbSvc.GetSLAMProcessConfig()
		cmd := append([]string{processCfg.Name}, processCfg.Args...)

		cmdResult := [][]string{
			{testExecutableName},
			{"-sensors="},
			{"-config_param={mode=mono,test_param=viam}", "-config_param={test_param=viam,mode=mono}"},
			{"-data_rate_ms=200"},
			{"-map_rate_sec=60"},
			{"-data_dir=" + name},
			{"-delete_processed_data=false"},
			{"-use_live_data=false"},
			{"-port=localhost:" + strconv.Itoa(port)},
			{"--aix-auto-update"},
		}

		for i, s := range cmd {
			t.Run(fmt.Sprintf("Test command argument %v at index %v", s, i), func(t *testing.T) {
				test.That(t, s, test.ShouldBeIn, cmdResult[i])
			})
		}

		grpcServer.Stop()
		test.That(t, utils.TryClose(context.Background(), svc), test.ShouldBeNil)
	})

	closeOutSLAMService(t, name)
}

func TestSLAMProcessFail(t *testing.T) {
	logger := golog.NewTestLogger(t)
	name, err := slamTesthelper.CreateTempFolderArchitecture(logger)
	test.That(t, err, test.ShouldBeNil)

	grpcServer, port := setupTestGRPCServer(t)
	attrCfg := &slamConfig.AttrConfig{
		Sensors:       []string{"good_color_camera"},
		ConfigParams:  map[string]string{"mode": "mono", "test_param": "viam"},
		DataDirectory: name,
		MapRateSec:    &validMapRate,
		DataRateMsec:  validDataRateMS,
		Port:          "localhost:" + strconv.Itoa(port),
		UseLiveData:   &_true,
	}

	t.Run("Run SLAM process that errors out due to invalid executable name", func(t *testing.T) {
		// This test ensures that we get the correct error if the user does
		// not have the correct binary installed.
		_, err := createSLAMService(t, attrCfg, logger, false, true, "fail_this_binary_does_not_exist")
		test.That(t, fmt.Sprint(err), test.ShouldContainSubstring, "executable file not found in $PATH")
	})

	grpcServer.Stop()

	closeOutSLAMService(t, name)
}
