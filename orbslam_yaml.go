// Package viamorbslam3 implements simultaneous localization and mapping
// This is an Experimental package
package viamorbslam3

import (
	"context"
	"io/fs"
	"os"
	"path/filepath"
	"strconv"
	"strings"
	"time"

	"github.com/pkg/errors"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/rimage/transform"
	"go.viam.com/rdk/utils"
	"go.viam.com/slam/dataprocess"
	"gopkg.in/yaml.v2"
)

const (
	// file version needed by ORBSLAM.
	fileVersion         = "1.0"
	yamlFilePrefixBytes = "%YAML:1.0\n"
)

// orbCamMaker takes in the camera properties and config params for orbslam and constructs a ORBsettings struct to use with yaml.Marshal.
func (orbSvc *orbslamService) orbCamMaker(camProperties *transform.PinholeCameraModel) (*ORBsettings, error) {
	var err error

	if camProperties.PinholeCameraIntrinsics == nil {
		return nil, transform.NewNoIntrinsicsError("Intrinsics do not exist")
	}
	intrinsics := camProperties.PinholeCameraIntrinsics
	orbslam := &ORBsettings{
		CamType:     "PinHole",
		Width:       intrinsics.Width,
		Height:      intrinsics.Height,
		Fx:          intrinsics.Fx,
		Fy:          intrinsics.Fy,
		Ppx:         intrinsics.Ppx,
		Ppy:         intrinsics.Ppy,
		FPSCamera:   int16(orbSvc.dataRateMs),
		FileVersion: fileVersion,
	}
	if orbSvc.dataRateMs <= 0 {
		// dataRateMs is always expected to be positive, since 0 gets reset to the default, and all other
		// values lower than the default are rejected
		return nil, errors.Errorf("orbslam yaml generation expected dataRateMs greater than 0, got %d", orbSvc.dataRateMs)
	}
	orbslam.FPSCamera = int16(1000 / orbSvc.dataRateMs)
	if orbslam.FPSCamera == 0 {
		orbslam.FPSCamera = 1
	}
	distortion, ok := camProperties.Distortion.(*transform.BrownConrady)
	if !ok {
		return nil, utils.NewUnimplementedInterfaceError(distortion, camProperties.Distortion)
	}
	orbslam.RadialK1 = distortion.RadialK1
	orbslam.RadialK2 = distortion.RadialK2
	orbslam.RadialK3 = distortion.RadialK3
	orbslam.TangentialP1 = distortion.TangentialP1
	orbslam.TangentialP2 = distortion.TangentialP2
	if orbslam.NFeatures, err = orbSvc.orbConfigToInt("orb_n_features", 1250); err != nil {
		return nil, err
	}
	if orbslam.ScaleFactor, err = orbSvc.orbConfigToFloat("orb_scale_factor", 1.2); err != nil {
		return nil, err
	}
	if orbslam.StereoThDepth, err = orbSvc.orbConfigToFloat("stereo_th_depth", 40); err != nil {
		return nil, err
	}
	if orbslam.DepthMapFactor, err = orbSvc.orbConfigToFloat("depth_map_factor", 1000); err != nil {
		return nil, err
	}
	if orbslam.NLevels, err = orbSvc.orbConfigToInt("orb_n_levels", 8); err != nil {
		return nil, err
	}
	if orbslam.IniThFAST, err = orbSvc.orbConfigToInt("orb_n_ini_th_fast", 20); err != nil {
		return nil, err
	}
	if orbslam.MinThFAST, err = orbSvc.orbConfigToInt("orb_n_min_th_fast", 7); err != nil {
		return nil, err
	}
	if orbslam.Stereob, err = orbSvc.orbConfigToFloat("stereo_b", 0.0745); err != nil {
		return nil, err
	}
	tmp, err := orbSvc.orbConfigToInt("rgb_flag", 0)
	if err != nil {
		return nil, err
	}
	orbslam.RGBflag = int8(tmp)

	return orbslam, nil
}

// ORBsettings is used to construct the yaml file.
type ORBsettings struct {
	FileVersion    string  `yaml:"File.version"`
	NFeatures      int     `yaml:"ORBextractor.nFeatures"`
	ScaleFactor    float64 `yaml:"ORBextractor.scaleFactor"`
	NLevels        int     `yaml:"ORBextractor.nLevels"`
	IniThFAST      int     `yaml:"ORBextractor.iniThFAST"`
	MinThFAST      int     `yaml:"ORBextractor.minThFAST"`
	CamType        string  `yaml:"Camera.type"`
	Width          int     `yaml:"Camera.width"`
	Height         int     `yaml:"Camera.height"`
	Fx             float64 `yaml:"Camera1.fx"`
	Fy             float64 `yaml:"Camera1.fy"`
	Ppx            float64 `yaml:"Camera1.cx"`
	Ppy            float64 `yaml:"Camera1.cy"`
	RadialK1       float64 `yaml:"Camera1.k1"`
	RadialK2       float64 `yaml:"Camera1.k2"`
	RadialK3       float64 `yaml:"Camera1.k3"`
	TangentialP1   float64 `yaml:"Camera1.p1"`
	TangentialP2   float64 `yaml:"Camera1.p2"`
	RGBflag        int8    `yaml:"Camera.RGB"`
	Stereob        float64 `yaml:"Stereo.b"`
	StereoThDepth  float64 `yaml:"Stereo.ThDepth"`
	DepthMapFactor float64 `yaml:"RGBD.DepthMapFactor"`
	FPSCamera      int16   `yaml:"Camera.fps"`
	LoadMapLoc     string  `yaml:"System.LoadAtlasFromFile"`
}

// orbGenYAML generates a .yaml file to be used with orbslam.
func (orbSvc *orbslamService) orbGenYAML(ctx context.Context, cam camera.Camera) error {
	// Get the camera and check if the properties are valid
	props, err := cam.Properties(ctx)
	if err != nil {
		return err
	}
	if props.IntrinsicParams == nil {
		return transform.NewNoIntrinsicsError("Intrinsics do not exist")
	}
	if err = props.IntrinsicParams.CheckValid(); err != nil {
		return err
	}
	if props.DistortionParams == nil {
		return transform.NewNoIntrinsicsError("Distortion parameters do not exist")
	}
	// create orbslam struct to generate yaml file with
	var cameraModel transform.PinholeCameraModel
	cameraModel.PinholeCameraIntrinsics = props.IntrinsicParams

	if props.DistortionParams != nil {
		cameraModel.Distortion = props.DistortionParams
	}
	orbslam, err := orbSvc.orbCamMaker(&cameraModel)
	if err != nil {
		return err
	}

	// Check for maps in the specified directory and add map to yaml config
	loadMapTimeStamp, loadMapName, err := orbSvc.checkMaps()
	if err != nil {
		orbSvc.logger.Debugf("Error occurred while parsing %s for maps, building map from scratch", orbSvc.dataDirectory)
	}
	if loadMapTimeStamp == "" {
		loadMapTimeStamp = time.Now().UTC().Format(dataprocess.SlamTimeFormat)
	} else {
		orbslam.LoadMapLoc = "\"" + loadMapName + "\""
	}

	// yamlFileName uses the timestamp from the loaded map if one was available
	// this gives the option to load images into the map if they were generated at a later time
	// orbslam also checks for the most recently generated yaml file to prevent any issues with timestamps here
	yamlFileName := filepath.Join(orbSvc.dataDirectory, "config", orbSvc.primarySensorName+"_data_"+loadMapTimeStamp+".yaml")

	// generate yaml file
	yamlData, err := yaml.Marshal(&orbslam)
	if err != nil {
		return errors.Wrap(err, "Error while Marshaling YAML file")
	}

	//nolint:gosec
	outfile, err := os.Create(yamlFileName)
	if err != nil {
		return err
	}

	if _, err = outfile.WriteString(yamlFilePrefixBytes); err != nil {
		return err
	}

	if _, err = outfile.Write(yamlData); err != nil {
		return err
	}
	return outfile.Close()
}

func (orbSvc *orbslamService) orbConfigToInt(key string, def int) (int, error) {
	valStr, ok := orbSvc.configParams[key]
	if !ok {
		orbSvc.logger.Debugf("Parameter %s not found, using default value %d", key, def)
		return def, nil
	}

	val, err := strconv.Atoi(valStr)
	if err != nil {
		return 0, errors.Errorf("Parameter %s has an invalid definition", key)
	}

	return val, nil
}

func (orbSvc *orbslamService) orbConfigToFloat(key string, def float64) (float64, error) {
	valStr, ok := orbSvc.configParams[key]
	if !ok {
		orbSvc.logger.Debugf("Parameter %s not found, using default value %f", key, def)
		return def, nil
	}

	val, err := strconv.ParseFloat(valStr, 64)
	if err != nil {
		return 0, errors.Errorf("Parameter %s has an invalid definition", key)
	}
	return val, nil
}

// checkMaps checks the map folder within the data directory for an existing map.
// It will grab the most recently generated map if one exists.
func (orbSvc *orbslamService) checkMaps() (string, string, error) {
	root := filepath.Join(orbSvc.dataDirectory, "map")
	mapExt := ".osa"
	mapTimestamp := time.Time{}
	var mapPath string

	err := filepath.WalkDir(root, func(path string, entry fs.DirEntry, err error) error {
		if !entry.IsDir() && filepath.Ext(path) == mapExt {
			// check if the file uses our format and grab timestamp if it does
			timestampLoc := strings.Index(entry.Name(), "_data_") + len("_data_")
			if timestampLoc != -1+len("_data_") {
				timestamp, err := time.Parse(dataprocess.SlamTimeFormat, entry.Name()[timestampLoc:strings.Index(entry.Name(), mapExt)])
				if err != nil {
					orbSvc.logger.Debugf("Unable to parse map %s, %v", path, err)
					return nil
				}
				if timestamp.After(mapTimestamp) {
					mapTimestamp = timestamp
					mapPath = path[0:strings.Index(path, mapExt)]
				}
			}
		}
		return nil
	})
	if err != nil {
		return "", "", err
	}
	// do not error out here, instead orbslam will build a map from scratch
	if mapTimestamp.IsZero() {
		orbSvc.logger.Debugf("No maps found in directory %s", root)
		return "", "", nil
	}
	orbSvc.logger.Infof("Previous map found, using %v", mapPath)
	return mapTimestamp.UTC().Format(dataprocess.SlamTimeFormat), mapPath, nil
}
