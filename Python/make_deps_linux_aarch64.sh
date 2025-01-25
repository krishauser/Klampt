#assumed to run in a Docker container containing Klampt directory
##assumed to run in a Docker container only containing Python directory

set -e  #error if any step fails

yum clean all
yum install -y mesa-libGLU-devel glpk glpk-dev
pip install cmake

#no glpk available on aarch64
# yum install -y libtool automake autoconf make
# git clone https://github.com/firedrakeproject/glpk.git
# pushd glpk
# autoreconf -f -i
# ./configure --disable-shared
# make CFLAGS='-fPIC -O3'
# make install
# popd

#git clone https://github.com/krishauser/Klampt.git

#pushd Klampt/Cpp/Dependencies
pushd Cpp/Dependencies
make unpack-deps
git clone https://github.com/assimp/assimp.git

pushd assimp
cmake -DASSIMP_BUILD_TESTS=OFF . && make 
make install
popd

pushd glew-2.1.0
make && make install
popd

make dep-tinyxml && make dep-ode

pushd KrisLibrary
cmake . -DC11_ENABLED=ON -DUSE_GLUT=OFF -DUSE_GLUI=OFF  && make 
popd

popd

#pushd Klampt
cmake . && make Klampt
make Pack
#bin/Pack  #somehow libGLEW.so.2.1 is not found
#popd

#echo "Working directory for cmake . call:"
#pwd

#update Python CMakeList from git pull'ed Klampt
#cmake -DKLAMPT_ROOT=Klampt -DKLAMPT_DEPENDENCIES=Klampt/Cpp/Dependencies Python
