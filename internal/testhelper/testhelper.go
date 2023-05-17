// Package testhelper implements a slam service definition with additional exported functions for
// the purpose of testing
package testhelper

import (
	"bufio"
	"context"
	"os"
	"testing"

	"github.com/edaniels/golog"
	"github.com/pkg/errors"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/test"
	"go.viam.com/utils/pexec"

	viamorbslam3 "github.com/viamrobotics/viam-orb-slam3"
	"github.com/viamrobotics/viam-orb-slam3/config"
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
		numFiles, err := CheckDataDirForExpectedFiles(t, dir+"/data/rgb", prev, deleteProcessedData, useLiveData)
		test.That(t, err, test.ShouldBeNil)
		return numFiles
	case viamorbslam3.Rgbd:
		numFilesRGB, err := CheckDataDirForExpectedFiles(t, dir+"/data/rgb", prev, deleteProcessedData, useLiveData)
		test.That(t, err, test.ShouldBeNil)

		numFilesDepth, err := CheckDataDirForExpectedFiles(t, dir+"/data/depth", prev, deleteProcessedData, useLiveData)
		test.That(t, err, test.ShouldBeNil)

		test.That(t, numFilesRGB, test.ShouldEqual, numFilesDepth)
		return numFilesRGB
	default:
		return 0
	}
}

const (
	dataBufferSize = 4
)

// CreateTempFolderArchitecture creates a new random temporary
// directory with the config, data, and map subdirectories needed
// to run the SLAM libraries.
func CreateTempFolderArchitecture(logger golog.Logger) (string, error) {
	tmpDir, err := os.MkdirTemp("", "*")
	if err != nil {
		return "nil", err
	}
	if err := config.SetupDirectories(tmpDir, logger); err != nil {
		return "", err
	}
	return tmpDir, nil
}

// ResetFolder removes all content in path and creates a new directory
// in its place.
func ResetFolder(path string) error {
	dirInfo, err := os.Stat(path)
	if err != nil {
		return err
	}
	if !dirInfo.IsDir() {
		return errors.Errorf("the path passed ResetFolder does not point to a folder: %v", path)
	}
	if err = os.RemoveAll(path); err != nil {
		return err
	}
	return os.Mkdir(path, dirInfo.Mode())
}

// CheckDataDirForExpectedFiles ensures that the provided data directory contains the correct amount of files
// based on the config parameters deleteProcessedData and useLiveData.
func CheckDataDirForExpectedFiles(t *testing.T, dir string, prev int, deleteProcessedData, useLiveData bool) (int, error) {
	files, err := os.ReadDir(dir)
	test.That(t, err, test.ShouldBeNil)

	if prev == 0 {
		return len(files), nil
	}
	if deleteProcessedData && useLiveData {
		test.That(t, prev, test.ShouldBeLessThanOrEqualTo, dataBufferSize+1)
	}
	if !deleteProcessedData && useLiveData {
		test.That(t, prev, test.ShouldBeLessThan, len(files))
	}
	if deleteProcessedData && !useLiveData {
		return 0, errors.New("the delete_processed_data value cannot be true when running SLAM in offline mode")
	}
	if !deleteProcessedData && !useLiveData {
		test.That(t, prev, test.ShouldEqual, len(files))
	}
	return len(files), nil
}
