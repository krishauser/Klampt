#!/bin/bash
set -e

cd /Klampt/Cpp/Dependencies/KrisLibrary; git checkout master; git pull; make -j 8
cd /Klampt; git pull; git checkout master; git pull; cmake .; make -j 8 Klampt

# Compile wheels
for PYBIN in /opt/python/*/bin; do
    echo "$PYBIN"
    "${PYBIN}/pip" wheel /Klampt/Python/ -w /wheelhouse/
done

# Bundle external shared libraries into the wheels
for whl in /wheelhouse/Klampt*.whl; do
    auditwheel repair --exclude libGLdispatch.so.0 --exclude libGLX.so.0 "$whl" -w /io/wheelhouse/
done

# Install packages and test
for PYBIN in /opt/python/*/bin/; do
    "${PYBIN}/pip" install PyOpenGL
    "${PYBIN}/pip" install numpy
    "${PYBIN}/pip" install Klampt --no-index -f /io/wheelhouse
    #(cd "$HOME"; "${PYBIN}/nosetests" pymanylinuxdemo)
done
