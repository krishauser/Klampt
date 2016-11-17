#!/bin/bash

cd Library
make deps-unpack

if [ "$TRAVIS_BRANCH" == "v0.7" ] || [ "$TRAVIS_BRANCH" == "travis" ]; then  
  cd KrisLibrary
  git checkout plan_devel
  cd ..
  echo "Checking out branch plan_devel for KrisLibrary"
fi

cd ..