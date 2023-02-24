# Viam ORB\_SLAM3 Module

## (In)stability Notice
> **Warning**
> This is an experimental feature. Stability is not guaranteed. Breaking changes are likely to occur, and occur often.

## Overview

ORB\_SLAM3 is a SLAM system for feature based mapping using monocular, rgbd, and stereo camera setups. 

For more information see [the official ORB_SLAM3 Repo](https://github.com/UZ-SLAMLab/ORB_SLAM3).

This is the [modular resource](https://docs.viam.com/program/extend/modular-resources/) code that wraps ORB\_SLAM3 so it is easily usable with the rest of Viam's ecosystem.

## Getting started

### Download 
```bash
git clone --recurse-submodules https://github.com:viamrobotics/viam-orb-slam3
```

If you happened to use `git clone` only, you won't see the `ORB_SLAM3` folder and will need to fetch it:

`git submodule update --init`

### Setup

To setup the gRPC files, run:

```bash
make bufinstall buf 
```

#### Automatic Dependency Installation (x64, arm64, or macOS)
To automatically install dependencies, use the target 
```bash
make setup
```

#### Manual Dependency Install (x64 or arm64)
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

### Building
Then to build:

```bash
make build
```

### Installing & Running

Currently, you can install and run this locally with
```bash
sudo cp ./viam-orb-slam3/bin/orb_grpc_server /usr/local/bin
```
```bash
orb_grpc_server
```
However in the future you will run this via the module system



## Development
### Linting

```bash
brew install clang-format
make format
```
### Testing

You can also run:

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
[submodule "slam-libraries/cartographer"]
        path = slam-libraries/cartographer
        url = git@github.com:kkufieta/cartographer.git
        branch=kk/fix-install
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
