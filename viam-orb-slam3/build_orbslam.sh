BASEDIR=`pwd`
echo $BASEDIR
ORBDIR=$BASEDIR/viam-orb-slam3
cd $BASEDIR/ORB_SLAM3/Thirdparty
ORB_THIRDPARTYDIR=`pwd`

echo "Configuring and building Thirdparty/DBoW2 ..."
#ORBSLAM used for place recognition
cd $ORB_THIRDPARTYDIR/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

echo "Configuring and building Thirdparty/g2o ..."
#ORBSLAM used for nonlinear optimization
cd $ORB_THIRDPARTYDIR/g2o
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

echo "Configuring and building Thirdparty/Sophus ..."
#ORBSLAM used for lie groups with 2D and 3D geometric problems
cd $ORB_THIRDPARTYDIR/Sophus
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

cd $BASEDIR/ORB_SLAM3

FILE=$BASEDIR/ORB_SLAM3/Vocabulary/ORBvoc.txt
if [ -f "$FILE" ]; then
    echo "Vocabulary already uncompressed."
else 
    echo "Uncompressing vocabulary..."
    tar -xf Vocabulary/ORBvoc.txt.tar.gz -C Vocabulary/
    echo "Do not forget to move vocabulary to your data directory!"
fi

echo "Configuring and building ORB_SLAM3..."
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j2

echo "Configuring and building viam-orb-slam3..."
cd $BASEDIR
mkdir bin
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j2
