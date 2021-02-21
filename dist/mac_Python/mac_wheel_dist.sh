#!/bin/bash
#run me from ~/Klampt/Python

export MACOSX_DEPLOYMENT_TARGET="10.9"

pyenv local 3.9.1
python -m pip install wheel
python -m pip install delocate twine
python setup.py build_ext
python setup.py install
python setup.py bdist_wheel

pyenv local 3.8.0
python -m pip install wheel
python setup.py build_ext
python setup.py install
python setup.py bdist_wheel

pyenv local 3.7.9
python -m pip install wheel
python setup.py build_ext
python setup.py install
python setup.py bdist_wheel

pyenv local 3.6.7
python -m pip install wheel
python setup.py build_ext
python setup.py install
python setup.py bdist_wheel

pyenv local 3.5.6
python -m pip install wheel
python setup.py build_ext
python setup.py install
python setup.py bdist_wheel

pyenv local 2.7.17
python -m pip install wheel
python setup.py build_ext
python setup.py install
python setup.py bdist_wheel

#here's where I installed delocate
pyenv local 3.9.1
pyenv rehash
delocate-wheel -w fixed_wheels -v dist/Klampt-0.8.6-cp27-cp27m-macosx_10_9_x86_64.whl
delocate-wheel -w fixed_wheels -v dist/Klampt-0.8.6-cp35-cp35m-macosx_10_9_x86_64.whl
delocate-wheel -w fixed_wheels -v dist/Klampt-0.8.6-cp36-cp36m-macosx_10_9_x86_64.whl
delocate-wheel -w fixed_wheels -v dist/Klampt-0.8.6-cp37-cp37m-macosx_10_9_x86_64.whl
delocate-wheel -w fixed_wheels -v dist/Klampt-0.8.6-cp38-cp38-macosx_10_9_x86_64.whl
delocate-wheel -w fixed_wheels -v dist/Klampt-0.8.6-cp39-cp39-macosx_10_9_x86_64.whl

echo "Upload using twine upload fixed_wheels/Klampt-0.8.6-*"
#twine upload fixed_wheels/Klampt-0.8.6-*
