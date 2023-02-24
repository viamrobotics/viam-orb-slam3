# ORB_SLAM3

## (In)stability Notice
> **Warning**
> This is an experimental feature. Stability is not guaranteed. Breaking changes are likely to occur, and occur often.

## Overview

ORB_SLAM3 is a SLAM system for feature based mapping using monocular, rgbd, and stereo camera setups. 

For more information see [the official ORB_SLAM3 Repo](https://github.com/UZ-SLAMLab/ORB_SLAM3).

## Installation instructions
Make sure to follow all steps as outlined in [the setup section here](../../README.md#setup) in addition to the steps below. 

### Automatic Dependency Installation (x64, arm64, or macOS)
To automatically install dependencies, use the target 
```
./setup_orbslam.sh
```
or
```
./setup_orbslam_macos.sh
```
### Manual Dependency Install (x64 or arm64)
```bash
# Install & build Pangolin (includes eigen)
git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin 
./scripts/install_prerequisites.sh recommended
mkdir build && cd build
cmake ..
make -j4 
sudo make install
```

```bash
# Install openCV
sudo apt install libopencv-dev
```

```bash
# Install Eigen3
sudo apt install libeigen3-dev
```

```bash
# Other dependencies
sudo apt install libssl-dev 
sudo apt-get install libboost-all-dev
```

### Build ORB_SLAM3
Ensure gRPC is setup by following the instructions described in this [README](../README.md). 

To build ORB_SLAM3 run the following
```bash
cd viam-orb-slam3
./build_orbslam.sh
```

This will build the binary and save it at `./bin/orb_grpc_server`. Move this binary into `usr/local/bin` by running:

```bash
sudo cp bin/orb_grpc_server /usr/local/bin/
```

In your desired data directory, move the vocabulary file from orbslam into your `~/config` folder:  
```bash
sudo cp ORB_SLAM3/Vocabulary/ORBvoc.txt ~/YOUR_DATA_DIR/config/
```
You only have to do this once per data directory. Note ORB_SLAM3 will fail if the Vocabulary cannot be found

To run tests, after building, run the following
```bash
cd viam-orb-slam3
./test_orbslam.sh
```
