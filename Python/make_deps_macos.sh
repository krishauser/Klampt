brew install assimp glew
brew uninstall --ignore-dependencies pkg-config
pip install cmake

pushd ../Cpp/Dependencies
make unpack-deps
make dep-tinyxml && make dep-ode
pushd KrisLibrary
git checkout devel
cmake . -DC11_ENABLED=ON -DUSE_GLUT=OFF -DUSE_GLUI=OFF -DCMAKE_OSX_ARCHITECTURES=\"arm64;x86_64\" && make -j
popd
popd

pushd ..
cmake . -DUSE_GLUT=OFF -DUSE_GLUI=OFF -DCMAKE_OSX_ARCHITECTURES=\"arm64;x86_64\" && make -j Klampt
popd
