set -e  #error if any step fails

brew install glew
brew uninstall --ignore-dependencies pkg-config
pip install cmake

ARCHFLAGS="-arch x86_64 -arch arm64 -isysroot /Applications/Xcode_12.4.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX11.1.sdk -mmacosx-version-min=10.9 -std=gnu++11"

pushd ../Cpp/Dependencies
make unpack-deps
git clone https://github.com/assimp/assimp.git
pushd assimp
cmake . -DCMAKE_OSX_ARCHITECTURES="arm64;x86_64"
make -j 
make install
popd
pushd tinyxml
make ARCHS="${ARCHFLAGS}"
popd
pushd ode-0.14;
X_EXTRA_LIBS=-lX11 CFLAGS="-fPIC ${ARCHFLAGS}" CXXFLAGS="-fPIC ${ARCHFLAGS}" ./configure --with-trimesh=none --disable-demos
make
popd
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
