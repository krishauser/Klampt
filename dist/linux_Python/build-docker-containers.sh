#!/bin/bash
set -e

docker pull quay.io/pypa/manylinux2010_x86_64
docker pull quay.io/pypa/manylinux2010_i686
docker build --no-cache -t manylinux_klampt_x86_64 -f Dockerfile_x86_64 .
docker build --no-cache -t manylinux_klampt_i686 -f Dockerfile_i686 .
