#!/bin/bash

if [[ $TRAVIS_BRANCH == 'v0.7' ]]
  cd Library
  make deps-unpack
  cd KrisLibrary
  git checkout plan_devel
  cd ..
else
  cd Library
  make deps-unpack
fi

cd