.PHONY: docs swig clean

default: cpp-docs swig docs

cpp-docs: 
	cd src; doxygen doxygen.conf
	cd src; ./doxy2swig.py docs/xml/index.xml docs/docs.i

docs:
	doxygen doxygen.conf

sphinx-docs:
	sphinx-apidoc --separate -o sphinx_docs .
	sphinx-build -b html sphinx_docs sphinx_docs/_build
	echo "HTML docs are placed in sphinx_docs/_build"

swig:
	swig -python -c++ src/robotsim.i
	swig -python -c++ src/motionplanning.i
	swig -python -c++ src/collide.i
	swig -python -c++ src/rootfind.i
	cp src/robotsim.py .
	cp src/motionplanning.py .
	cp src/collide.py .
	cp src/rootfind.py .


clean:
	rm -f *.pyc
	rm -f doxygen_warnings.log
	rm -rf docs



