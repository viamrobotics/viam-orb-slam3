// Package main provides an ORB_SLAM3 implementation of a SLAM module
package main

import (
	"context"

	"github.com/edaniels/golog"
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/services/slam"
	"go.viam.com/utils"

	viamorbslam3 "github.com/viamrobotics/viam-orb-slam3"
)

func main() {
	utils.ContextualMain(mainWithArgs, golog.NewLogger("orbslam3Module"))
}

func mainWithArgs(ctx context.Context, args []string, logger golog.Logger) error {
	// Instantiate the module
	orbModule, err := module.NewModuleFromArgs(ctx, logger)
	if err != nil {
		return err
	}

	// Add model to the module
	if err = orbModule.AddModelFromRegistry(ctx, slam.API, viamorbslam3.Model); err != nil {
		return err
	}

	// Start the module
	err = orbModule.Start(ctx)
	defer orbModule.Close(ctx)
	if err != nil {
		return err
	}
	<-ctx.Done()
	return nil
}
