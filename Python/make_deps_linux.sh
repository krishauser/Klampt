#assumed to run in a Docker container containing Klampt directory
##assumed to run in a Docker container only containing Python directory

set -e  #error if any step fails

yum clean all
yum install -y glpk glpk-devel mesa-libGLU-devel git  
pip install cmake

#git clone https://github.com/krishauser/Klampt.git

#pushd Klampt/Cpp/Dependencies
pushd Cpp/Dependencies
make unpack-deps
git clone  --branch v5.2.5 --depth 1 https://github.com/assimp/assimp.git

pushd assimp
cmake -DASSIMP_BUILD_TESTS=OFF . && make -j && make install
popd

pushd glew-2.1.0
make -j && make install
popd

make dep-tinyxml && make dep-ode

pushd KrisLibrary
cmake . -DC11_ENABLED=ON -DUSE_GLUT=OFF -DUSE_GLUI=OFF  && make
popd

popd

#pushd Klampt
cmake . && make -j Klampt
make -j Pack
bin/Pack
#popd

#echo "Working directory for cmake . call:"
#pwd

#update Python CMakeList from git pull'ed Klampt
#cmake -DKLAMPT_ROOT=Klampt -DKLAMPT_DEPENDENCIES=Klampt/Cpp/Dependencies Python
