#!/bin/bash
set -e

#docker pull quay.io/pypa/manylinux2010_x86_64
#docker pull quay.io/pypa/manylinux2010_i686
docker pull quay.io/pypa/manylinux2014_x86_64
#docker pull quay.io/pypa/manylinux2014_i686
#docker pull quay.io/pypa/manylinux2014_aarch64
#docker pull quay.io/pypa/manylinux_2_28_x86_64
#docker pull quay.io/pypa/manylinux_2_28_i686
#docker pull quay.io/pypa/manylinux_2_28_aarch64
docker build --no-cache -t manylinux_klampt_x86_64 -f Dockerfile_x86_64 .
#docker build --no-cache -t manylinux_klampt_i686 -f Dockerfile_i686 .
#docker build --no-cache --platform=linux/arm64 -t manylinux_klampt_aarch64 -f Dockerfile_aarch64 .
