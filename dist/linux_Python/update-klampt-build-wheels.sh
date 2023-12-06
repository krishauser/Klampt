#!/bin/bash
set -e

cd /Klampt/Cpp/Dependencies/KrisLibrary; git checkout master; git pull; make -j 8
cd /Klampt; git pull; git checkout master; git pull; make -j 8 Klampt
cp /Klampt/Python/setup.py /io/wheelhouse

# Compile wheels
for PYBIN in /opt/python/*/bin; do
    echo "$PYBIN"
    "${PYBIN}/pip" wheel /Klampt/Python/ -w /wheelhouse/
done

# Bundle external shared libraries into the wheels
for whl in /wheelhouse/Klampt*.whl; do
    auditwheel repair --exclude libGLX.so.0 --exclude libGLdispatch.so.0 --exclude libOpenGL.so.0 "$whl" -w /io/wheelhouse/
done

# Install packages and test
for PYBIN in /opt/python/*/bin/; do
    "${PYBIN}/pip" install Klampt --no-index -f /io/wheelhouse
    #(cd "$HOME"; "${PYBIN}/nosetests" pymanylinuxdemo)
done
