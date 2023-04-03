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

	// log.Println("ZACK--STARTING DIRECTLY")
    // appDir := os.Getenv("APPDIR")
    // cwd,err := os.Getwd()
	// if err != nil {
	// 	fmt.Println(err)
	// }

    // err = os.Chdir(os.Getenv("APPRUN_RUNTIME"))
	// if err != nil {
	// 	fmt.Println("Can't switch AppImage runtime, further errors may be encountered.")
	// 	fmt.Println(err)
	// }
    // target := "/usr/local/bin/orb_grpc_server"

	// if appDir != "" {
	// 	target = appDir + "/" + strings.TrimPrefix(target, "/")
	// }

	// newArgs := []string{}


    // p:= pexec.ProcessConfig{
	// 	ID:      "slam_orbslam3",
	// 	Name:    target,
	// 	Args:    newArgs,
	// 	Log:     true,
	// 	OneShot: false,
	// }
    // slamProcess := pexec.NewProcessManager(logger)

	// _, err = slamProcess.AddProcessFromConfig(ctx,p)
	// if err != nil {
        // panic(err.Error())
	// }

	// if err = slamProcess.Start(ctx); err != nil {
	// 	panic(err.Error())
	//  }

    // err = os.Chdir(cwd)
	// if err != nil {
	// 	fmt.Println("Can't switch AppImage runtime, further errors may be encountered.")
	// 	fmt.Println(err)
	// }
    // time.Sleep(25 * time.Second)

	// if err = slamProcess.Stop(); err != nil {
	// 	panic(err.Error())
	//  }

	// println("Parent dies now.")
	// log.Println("ZACK--DONE DIRECTLY")

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
