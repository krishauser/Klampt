#!/bin/bash
set -e

docker run --rm -v `pwd`:/io manylinux_klampt_x86_64:latest /io/update-klampt-build-wheels.sh
docker run --rm -v `pwd`:/io manylinux_klampt_i686:latest linux32 /io/update-klampt-build-wheels.sh
ls wheelhouse/

