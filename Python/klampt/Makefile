.PHONY: docs swig clean

default: docs cpp-docs swig

cpp-docs: 
	cd src; doxygen doxygen.conf
	cd src; python doxy2swig.py -w 80 docs/xml/index.xml docs/docs.i

docs:
	doxygen doxygen.conf

sphinx-docs:
	sphinx-build -b html ../docs/source ../docs/build
	echo "HTML docs are placed in Klampt/Python/docs/build/html"

swig:
	swig -python -py3 -c++ src/robotsim.i
	swig -python -py3 -c++ src/motionplanning.i
	swig -python -py3 -c++ src/rootfind.i
	python3 src/improve_swig_docs.py src/robotsim.py > src/robotsim_temp.py
	python3 src/improve_swig_docs.py src/motionplanning.py > src/motionplanning_temp.py
	python3 src/improve_swig_docs.py src/rootfind.py > src/rootfind_temp.py
	cp src/robotsim_temp.py robotsim.py
	cp src/motionplanning_temp.py plan/motionplanning.py
	cp src/rootfind_temp.py math/rootfind.py

clean:
	rm -f *.pyc
	rm -f doxygen_warnings.log
	rm -rf ../docs/sphinx/build
	rm -rf src/docs



