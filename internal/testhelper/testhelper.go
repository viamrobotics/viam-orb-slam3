// Package testhelper implements a slam service definition with additional exported functions for
// the purpose of testing
package testhelper

import (
	"bufio"
	"context"
	"testing"

	"go.viam.com/rdk/components/camera"
	slamTesthelper "go.viam.com/slam/testhelper"
	"go.viam.com/test"
	"go.viam.com/utils/pexec"

	viamorbslam3 "github.com/viamrobotics/viam-orb-slam3"
)

// Service in the internal package includes additional exported functions relating to the data and
// slam processes in the slam service. These functions are not exported to the user. This resolves
// a circular import caused by the inject package.
type Service interface {
	StartDataProcess(cancelCtx context.Context, cam []camera.Camera, c chan int)
	StartSLAMProcess(ctx context.Context) error
	StopSLAMProcess() error
	Close(ctx context.Context) error
	GetSLAMProcessConfig() pexec.ProcessConfig
	GetSLAMProcessBufferedLogReader() bufio.Reader
}

// CheckDeleteProcessedData compares the number of files found in a specified data
// directory with the previous number found and uses the useLiveData and
// deleteProcessedData values to evaluate this comparison. It returns the number of files
// currently in the data directory for the specified config. Future invocations should pass in this
// value. This function should be passed 0 as a default prev argument in order to get the
// number of files currently in the directory.
func CheckDeleteProcessedData(
	t *testing.T,
	subAlgo viamorbslam3.SubAlgo,
	dir string,
	prev int,
	deleteProcessedData,
	useLiveData bool,
) int {
	switch subAlgo {
	case viamorbslam3.Mono:
		numFiles, err := slamTesthelper.CheckDataDirForExpectedFiles(t, dir+"/data/rgb", prev, deleteProcessedData, useLiveData)
		test.That(t, err, test.ShouldBeNil)
		return numFiles
	case viamorbslam3.Rgbd:
		numFilesRGB, err := slamTesthelper.CheckDataDirForExpectedFiles(t, dir+"/data/rgb", prev, deleteProcessedData, useLiveData)
		test.That(t, err, test.ShouldBeNil)

		numFilesDepth, err := slamTesthelper.CheckDataDirForExpectedFiles(t, dir+"/data/depth", prev, deleteProcessedData, useLiveData)
		test.That(t, err, test.ShouldBeNil)

		test.That(t, numFilesRGB, test.ShouldEqual, numFilesDepth)
		return numFilesRGB
	default:
		return 0
	}
}
