# viam-orb-slam3

## (In)stability Notice
> **Warning**
> This is an experimental feature. Stability is not guaranteed. Breaking changes are likely to occur, and occur often.

## Overview

This repo wraps [ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) as a [modular resource](https://docs.viam.com/program/extend/modular-resources/) so it is easily usable with the rest of Viam's ecosystem. ORB_SLAM3 is a SLAM system for feature-based mapping using monocular, rgbd, and stereo camera setups. 

## Getting started

Install viam-orb-slam3:

* Linux aarch64:
    ```bash
    sudo curl -o /usr/local/bin/orb_grpc_server http://packages.viam.com/apps/slam-servers/orb_grpc_server-stable-aarch64.AppImage
    sudo chmod a+rx /usr/local/bin/orb_grpc_server
    ```
 * Linux x86_64:
    ```bash
    sudo curl -o /usr/local/bin/orb_grpc_server http://packages.viam.com/apps/slam-servers/orb_grpc_server-stable-x86_64.AppImage
    sudo chmod a+rx /usr/local/bin/orb_grpc_server
    ```

* Homebrew / Linuxbrew
    ```bash
    brew tap viamrobotics/brews && brew install orb-grpc-server
    ```

For next steps, see the [Run ORB-SLAM3 on your Robot with a Webcam Tutorial](https://docs.viam.com/services/slam/run-slam-webcam/).

## Development

### Download 
```bash
git clone --recurse-submodules https://github.com:viamrobotics/viam-orb-slam3
```

If you happened to use `git clone` only, you won't see the `ORB_SLAM3` folder and will need to fetch it:

`git submodule update --init`

### (Optional) Using Canon Images

If desired, Viam's canon tool can be used to create a docker container to build `arm64` or `amd64` binaries of the SLAM server. The canon tool can be installed by running the following command: 

```bash
go install github.com/viamrobotics/canon@latest
```

And then by running one of the following commands in the viam-orb-slam3 repository to create the container:

```bash
canon -arch arm64
```

```bash
canon -arch amd64
```

These containers are set to persist between sessions via the `persistent` parameter in the `.canon.yaml` file located in the root of viam-orb-slam3. More details regarding the use of Viam's canon tool can be found [here](https://github.com/viamrobotics/canon).

### Setup, build, and run the binary

```bash
# Setup the gRPC files
make bufinstall buf 
# Install dependencies
make setup
# Build & install the binary
make build
sudo cp ./viam-orb-slam3/bin/orb_grpc_server /usr/local/bin
# Run the binary
orb_grpc_server
```

#### Alternative: Manual Dependency Install (x64 or arm64)
```bash
# Install & build Pangolin (includes eigen)
git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin 
./scripts/install_prerequisites.sh recommended
mkdir build && cd build
cmake ..
make -j4 
sudo make install
# Install openCV
sudo apt install libopencv-dev
# Install Eigen3
sudo apt install libeigen3-dev
# Other dependencies
sudo apt install libssl-dev 
sudo apt-get install libboost-all-dev
```

### Linting

```bash
# Ubuntu:
sudo apt install clang-format
# macOS:
brew install clang-format
make format
```

### Testing

```bash
make test
```

### Working with submodules

#### Commit and push
1. Commit and push changes in the submodules first.
2. Commit and push changes in the `slam` library last.

Or, alternatively:
1. Commit changes in the submodules
1. Commit changes in the main repo
1. Push all changes by running `git push --recurse-submodules=on-demand`

#### Changing branches in a submodule
When changing branches in a submodule, update `.gitmodules`, e.g., changing to a branch called `kk/fix-install`:

```bash
...
[submodule "viam-orb-slam3/ORB_SLAM3"]
        path = viam-orb-slam3/ORB_SLAM3
        url = git@github.com:kkufieta/ORB_SLAM3.git
        branch=kk/fix-this-bug
```

Commit & push the changes.

When pulling those changes, run the following:
```bash
git pull
git submodule update --init --recursive
```

## License
Copyright 2023 Viam Inc.

Apache 2.0 - See [LICENSE](https://github.com/viamrobotics/slam/blob/main/LICENSE) file
