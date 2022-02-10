set -e  #error if any step fails

brew install glew
brew uninstall --ignore-dependencies pkg-config
pip install cmake

pushd ../Cpp/Dependencies
make unpack-deps
git clone https://github.com/assimp/assimp.git
pushd assimp
cmake . -DCMAKE_OSX_ARCHITECTURES="arm64;x86_64"
make -j 
make install
popd
pushd tinyxml
make ARCHS="-arch x86_64 -arch arm64"
popd
make dep-ode
pushd KrisLibrary
git checkout devel
cmake . -DC11_ENABLED=ON -DUSE_GLUT=OFF -DUSE_GLUI=OFF -DCMAKE_OSX_ARCHITECTURES="arm64;x86_64"
make -j
popd
popd

pushd ..
cmake . -DUSE_GLUT=OFF -DUSE_GLUI=OFF -DCMAKE_OSX_ARCHITECTURES="arm64;x86_64"
make -j Klampt
popd
