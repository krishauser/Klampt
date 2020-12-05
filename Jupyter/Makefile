.PHONY: install install-user install-wurlitzer

all: install
	echo "done";

install:
	jupyter nbextension install klampt/
	jupyter nbextension enable klampt/main
	jupyter nbextension enable klampt/three.min
	jupyter nbextension enable klampt/KlamptFrontend

install-user:
	jupyter nbextension install klampt/ --user
	jupyter nbextension enable klampt/main 
	jupyter nbextension enable klampt/three.min 
	jupyter nbextension enable klampt/KlamptFrontend 

install-wurlitzer:
	git clone https://github.com/minrk/wurlitzer.git
	cd wurlitzer; python setup.py install
