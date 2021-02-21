SWIG=swig
#SWIG=/usr/local/bin/swig
PYTHON=python3

.PHONY: docs swig clean

default: docs cpp-docs swig

cpp-docs: 
	cd src; doxygen doxygen.conf
	cd src; $(PYTHON) doxy2swig.py -w 80 docs/xml/index.xml docs/docs.i

docs:
	doxygen doxygen.conf

sphinx-docs:
	sphinx-build -b html ../docs/source ../docs/build
	echo "HTML docs are placed in Klampt/Python/docs/build/html"

swig:
	$(SWIG) -python -c++ src/robotsim.i
	$(SWIG) -python -c++ src/motionplanning.i
	$(SWIG) -python -c++ src/rootfind.i
	$(PYTHON) src/improve_swig_docs.py src/robotsim.py > src/robotsim_temp.py
	$(PYTHON) src/improve_swig_docs.py src/motionplanning.py > src/motionplanning_temp.py
	$(PYTHON) src/improve_swig_docs.py src/rootfind.py > src/rootfind_temp.py
	cp src/robotsim_temp.py robotsim.py
	cp src/motionplanning_temp.py plan/motionplanning.py
	cp src/rootfind_temp.py math/rootfind.py

clean:
	rm -f *.pyc
	rm -f doxygen_warnings.log
	rm -rf ../docs/sphinx/build
	rm -rf src/docs



