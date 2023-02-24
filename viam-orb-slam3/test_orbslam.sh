#!/bin/bash
BASEDIR=`pwd`
echo $BASEDIR

echo "Running tests..."
cd $BASEDIR
./bin/orb_grpc_server_test
