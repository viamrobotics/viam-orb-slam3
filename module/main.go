// Package main provides an ORB_SLAM3 implementation of a SLAM module
package main

import (
	"context"

	"github.com/edaniels/golog"
	viamorbslam3 "github.com/viamrobotics/viam-orb-slam3"
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/services/slam"
	"go.viam.com/utils"
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
    if err = orbModule.AddModelFromRegistry(ctx, slam.Subtype, viamorbslam3.Model); err != nil {
        return err
    }


    // Start the module
    // TODO(@kat) -- why does the rplidar module 
    // do the defer before error checking? 
    // https://github.com/viamrobotics/rplidar/blob/main/module/main.go
    if err = orbModule.Start(ctx); err != nil {
        return err
    }
    defer orbModule.Close(ctx)
    <- ctx.Done()
    return nil
}
