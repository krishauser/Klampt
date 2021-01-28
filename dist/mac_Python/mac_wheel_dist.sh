#!/bin/bash
#run me from ~/Klampt/Python
pyenv local 3.9.1
pip install wheel
python setup.py build_ext
python setup.py install
python setup.py bdist_wheel

pyenv local 3.8.0
pip install wheel
python setup.py build_ext
python setup.py install
python setup.py bdist_wheel

pyenv local 3.7.9
pip install wheel
python setup.py build_ext
python setup.py install
python setup.py bdist_wheel

pyenv local 3.6.7
pip install wheel
python setup.py build_ext
python setup.py install
python setup.py bdist_wheel

pyenv local 3.5.6
pip install wheel
python setup.py build_ext
python setup.py install
python setup.py bdist_wheel

pyenv local 2.7.17
pip install wheel
python setup.py build_ext
python setup.py install
python setup.py bdist_wheel

#here's where I installed delocate
pyenv local 3.5.6
delocate-wheel -w fixed_wheels -v dist/Klampt-0.8.6-cp27-cp27m-macosx_10_9_x86_64.whl
delocate-wheel -w fixed_wheels -v dist/Klampt-0.8.6-cp34-cp34m-macosx_10_9_x86_64.whl
delocate-wheel -w fixed_wheels -v dist/Klampt-0.8.6-cp35-cp35m-macosx_10_9_x86_64.whl
delocate-wheel -w fixed_wheels -v dist/Klampt-0.8.6-cp36-cp36m-macosx_10_9_x86_64.whl
delocate-wheel -w fixed_wheels -v dist/Klampt-0.8.6-cp37-cp37m-macosx_10_9_x86_64.whl
delocate-wheel -w fixed_wheels -v dist/Klampt-0.8.6-cp38-cp38m-macosx_10_9_x86_64.whl
delocate-wheel -w fixed_wheels -v dist/Klampt-0.8.6-cp39-cp39m-macosx_10_9_x86_64.whl

#heres where i installed twine
#pyenv local 3.8.0
#twine upload fixed_wheels/*
