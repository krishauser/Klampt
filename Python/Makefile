.PHONY: inplace install lib docs

default: install

inplace:
	python setup.py build_ext --inplace

lib:
	python setup.py build_ext

install:
	python setup.py install

docs:
	cd klampt; make docs

