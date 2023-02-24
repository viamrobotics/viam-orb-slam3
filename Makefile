BUILD_CHANNEL?=local

bufinstall:
	sudo apt-get install -y protobuf-compiler-grpc libgrpc-dev libgrpc++-dev || brew install grpc openssl --quiet

bufsetup:
	GOBIN=`pwd`/grpc/bin go install github.com/bufbuild/buf/cmd/buf@v1.8.0
	ln -sf `which grpc_cpp_plugin` grpc/bin/protoc-gen-grpc-cpp
	pkg-config openssl || export PKG_CONFIG_PATH=$$PKG_CONFIG_PATH:`find \`which brew > /dev/null && brew --prefix\` -name openssl.pc | head -n1 | xargs dirname`

buf: bufsetup
	PATH="${PATH}:`pwd`/grpc/bin" buf generate --template ./buf.gen.yaml buf.build/viamrobotics/api
	PATH="${PATH}:`pwd`/grpc/bin" buf generate --template ./buf.grpc.gen.yaml buf.build/viamrobotics/api
	PATH="${PATH}:`pwd`/grpc/bin" buf generate --template ./buf.gen.yaml buf.build/googleapis/googleapis

clean:
	rm -rf grpc
	rm -rf viam-orb-slam3/build
	rm -rf viam-orb-slam3/ORB_SLAM3/build
	rm -rf viam-orb-slam3/ORB_SLAM3/lib
	rm -rf viam-orb-slam3/ORB_SLAM3/Thirdparty/DBoW2/build
	rm -rf viam-orb-slam3/ORB_SLAM3/Thirdparty/g2o/build
	rm -rf viam-orb-slam3/ORB_SLAM3/Thirdparty/Sophus/build
	rm -rf viam-orb-slam3/bin

format-setup:
	sudo apt-get install -y clang-format

format:
	find . -type f -not -path \
		-and ! -path '*viam-cartographer/cartographer*' \
		-and ! -path '*viam-cartographer/build*' \
		-and ! -path '*viam-orb-slam3/ORB_SLAM3*' \
		-and ! -path '*api*' \
		-and \( -iname '*.h' -o -iname '*.cpp' -o -iname '*.cc' \) \
		| xargs clang-format -i --style="{BasedOnStyle: Google, IndentWidth: 4}"

setup:
ifeq ("Darwin", "$(shell uname -s)")
	cd viam-orb-slam3 && ./setup_orbslam_macos.sh
else
	cd viam-orb-slam3 && ./setup_orbslam.sh
endif

build:
	cd viam-orb-slam3 && ./build_orbslam.sh

test:
	cd viam-orb-slam3 && ./test_orbslam.sh

all: bufinstall buf setup build test


appimage: build
	cd etc/packaging/appimages && BUILD_CHANNEL=${BUILD_CHANNEL} appimage-builder --recipe orb_grpc_server-`uname -m`.yml
	cd etc/packaging/appimages && ./package_release_orb.sh
	mkdir -p etc/packaging/appimages/deploy/
	mv etc/packaging/appimages/*.AppImage* etc/packaging/appimages/deploy/
	chmod 755 etc/packaging/appimages/deploy/*.AppImage

include *.make
