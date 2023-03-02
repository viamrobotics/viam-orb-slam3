BUILD_CHANNEL?=local
TOOL_BIN = bin/gotools/$(shell uname -s)-$(shell uname -m)
PATH_WITH_TOOLS="`pwd`/$(TOOL_BIN):${PATH}"

bufinstall:
	sudo apt-get install -y protobuf-compiler-grpc libgrpc-dev libgrpc++-dev || brew install grpc openssl --quiet

bufsetup:
	GOBIN=`pwd`/grpc/bin go install github.com/bufbuild/buf/cmd/buf@v1.8.0
	ln -sf `which grpc_cpp_plugin` grpc/bin/protoc-gen-grpc-cpp
	pkg-config openssl || export PKG_CONFIG_PATH=$$PKG_CONFIG_PATH:`find \`which brew > /dev/null && brew --prefix\` -name openssl.pc | head -n1 | xargs dirname`

buf: bufsetup
	PATH="${PATH}:`pwd`/grpc/bin" buf generate --template ./buf/buf.gen.yaml buf.build/viamrobotics/api
	PATH="${PATH}:`pwd`/grpc/bin" buf generate --template ./buf/buf.grpc.gen.yaml buf.build/viamrobotics/api
	PATH="${PATH}:`pwd`/grpc/bin" buf generate --template ./buf/buf.gen.yaml buf.build/googleapis/googleapis

clean:
	rm -rf grpc
	rm -rf viam-orb-slam3/build
	rm -rf viam-orb-slam3/ORB_SLAM3/build
	rm -rf viam-orb-slam3/ORB_SLAM3/lib
	rm -rf viam-orb-slam3/ORB_SLAM3/Thirdparty/DBoW2/build
	rm -rf viam-orb-slam3/ORB_SLAM3/Thirdparty/g2o/build
	rm -rf viam-orb-slam3/ORB_SLAM3/Thirdparty/Sophus/build
	rm -rf viam-orb-slam3/bin

ensure-submodule-initialized:
	@if [ ! -d "viam-orb-slam3/ORB_SLAM3/src" ]; then \
		echo "Submodule was not initialized. Initializing..."; \
		git submodule update --init; \
	else \
		echo "Submodule found successfully"; \
	fi 

lint-setup-cpp:
ifeq ("Darwin", "$(shell uname -s)")
	brew install clang-format
else
	sudo apt-get install -y clang-format
endif

lint-setup-go:
	GOBIN=`pwd`/$(TOOL_BIN) go install \
		github.com/edaniels/golinters/cmd/combined \
		github.com/golangci/golangci-lint/cmd/golangci-lint \
		github.com/rhysd/actionlint/cmd/actionlint

lint-setup: lint-setup-cpp lint-setup-go

lint-go: lint-setup-go
	go vet -vettool=$(TOOL_BIN)/combined ./...
	GOGC=50 $(TOOL_BIN)/golangci-lint run -v --fix --config=./etc/golangci.yaml
	PATH=$(PATH_WITH_TOOLS) actionlint

lint-cpp: lint-setup-cpp
	find . -type f -not -path \
		-and ! -path '*viam-orb-slam3/ORB_SLAM3*' \
		-and ! -path '*api*' \
		-and ! -path '*grpc*' \
		-and \( -iname '*.h' -o -iname '*.cpp' -o -iname '*.cc' \) \
		| xargs clang-format -i --style="{BasedOnStyle: Google, IndentWidth: 4}"

lint: ensure-submodule-initialized lint-go lint-cpp

setup: ensure-submodule-initialized
ifeq ("Darwin", "$(shell uname -s)")
	cd viam-orb-slam3 && ./scripts/setup_orbslam_macos.sh
else
	cd viam-orb-slam3 && ./scripts/setup_orbslam_linux.sh
endif

build:
	cd viam-orb-slam3 && ./scripts/build_orbslam.sh

test-module-wrapper:
	go test -race ./...

test-core:
	cd viam-orb-slam3 && ./scripts/test_orbslam.sh

test: test-core test-module-wrapper

install:
	sudo cp viam-orb-slam3/bin/orb_grpc_server /usr/local/bin/orb_grpc_server

appimage: build
	cd etc/packaging/appimages && BUILD_CHANNEL=${BUILD_CHANNEL} appimage-builder --recipe orb_grpc_server-`uname -m`.yml
	cd etc/packaging/appimages && ./package_release_orb.sh
	mkdir -p etc/packaging/appimages/deploy/
	mv etc/packaging/appimages/*.AppImage* etc/packaging/appimages/deploy/
	chmod 755 etc/packaging/appimages/deploy/*.AppImage

include *.make
