echo "Installing ORB_SLAM3 external dependencies"
brew tap viamrobotics/brews
brew install cmake glew opencv@4 eigen boost openssl pangolin
brew link openssl --force
sudo ln -s `which brew > /dev/null && brew --prefix`/Cellar/pangolin/0.8/lib/*.dylib /usr/local/lib
