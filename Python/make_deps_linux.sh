#assumed to run in a Docker container only containing Python directory

set -e  #error if any step fails

yum clean all
yum install -y glpk glpk-devel mesa-libGLU-devel git
pip install cmake

git clone https://github.com/krishauser/Klampt.git

pushd Klampt/Cpp/Dependencies
make unpack-deps
git clone https://github.com/assimp/assimp.git

pushd assimp
cmake -DASSIMP_BUILD_TESTS=OFF . && make -j && make install
popd

pushd glew-2.1.0
make -j && make install
popd

make dep-tinyxml && make dep-ode

pushd KrisLibrary
cmake . -DC11_ENABLED=ON -DUSE_GLUT=OFF -DUSE_GLUI=OFF  && make -j
popd

popd

pushd Klampt
cmake . && make -j Klampt
make -j Pack
bin/Pack
popd

#update Python CMakeList from git pull'ed Klampt
cmake -DKLAMPT_ROOT=Klampt -DKRISLIBRARY_ROOT=Klampt/Cpp/Dependencies .
