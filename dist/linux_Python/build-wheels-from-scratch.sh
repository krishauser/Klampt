#!/bin/bash
set -e

yum install -y git gcc glpk glpk-devel mesa-libGLU-devel

cd io/cmake-2.8.12
./bootstrap; make -j 8; make install
cd /

git clone https://github.com/assimp/assimp.git
cd assimp; cmake .; make -j 8; make install

cd /
git clone https://github.com/krishauser/Klampt
cd /Klampt; git checkout master

cd /Klampt/Cpp/Dependencies; make unpack-deps
#take out the sudo call
cd glew-2.0.0; make -j 8; make install
cd ..
make dep-tinyxml
make dep-ode
cd KrisLibrary; cmake . -DC11_ENABLED=ON; make -j 8

cd /Klampt
cmake .; make -j 8 Klampt
cd /

# Compile wheels
for PYBIN in /opt/python/*/bin; do
    "${PYBIN}/pip" wheel /Klampt/Python/ -w /io/wheelhouse/
done

# Bundle external shared libraries into the wheels
for whl in io/wheelhouse/*.whl; do
    auditwheel repair "$whl" -w /io/wheelhouse/
done

# Install packages and test
for PYBIN in /opt/python/*/bin/; do
    "${PYBIN}/pip" install python-manylinux-demo --no-index -f /io/wheelhouse
    #(cd "$HOME"; "${PYBIN}/nosetests" pymanylinuxdemo)
done