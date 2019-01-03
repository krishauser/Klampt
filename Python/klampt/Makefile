.PHONY: docs swig clean python3_version

default: cpp-docs swig docs

cpp-docs: 
	cd src; doxygen doxygen.conf
	cd src; python doxy2swig.py -w 80 docs/xml/index.xml docs/docs.i

docs:
	doxygen doxygen.conf

sphinx-docs:
	sphinx-build -b html ../docs/source ../docs/build
	echo "HTML docs are placed in Klampt/Python/docs/build/html"

swig:
	swig -python -c++ src/robotsim.i
	swig -python -c++ src/motionplanning.i
	swig -python -c++ src/rootfind.i
	python src/improve_swig_docs.py src/robotsim.py > src/robotsim_temp.py
	python src/improve_swig_docs.py src/motionplanning.py > src/motionplanning_temp.py
	python src/improve_swig_docs.py src/rootfind.py > src/rootfind_temp.py
	cp src/robotsim_temp.py robotsim.py
	cp src/motionplanning_temp.py plan/motionplanning.py
	cp src/rootfind_temp.py math/rootfind.py

python3_version:
	cp -r data ../python3_version/klampt/
	2to3 --output-dir=../python3_version/klampt -W -n .
	2to3 klampt_browser -n -w -o ../python3_version/bin
	2to3 klampt_path -n -w -o ../python3_version/bin
	2to3 klampt_sim -n -w -o ../python3_version/bin
	2to3 klampt_thumbnails -n -w -o ../python3_version/bin

clean:
	rm -f *.pyc
	rm -f doxygen_warnings.log
	rm -rf ../docs/sphinx/build
	rm -rf src/docs



