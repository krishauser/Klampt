TARGETS=WebServer
#-fPIC
CXXFLAGS +=  -w -fpermissive -Wwrite-strings
CC=g++

CXXFLAGS += $(shell python2.7-config --includes)
LDFLAGS +=  $(shell python2.7-config --libs)

all: $(TARGETS)

WebServer: main.o websocket.o PythonWrapper.o
	$(CC)  $^ -lssl -lcrypto -lresolv $(LDFLAGS) -o $@
	cp WebServer ../..

websocket.o: websocket.cpp websocket.h
main.o: main.cpp websocket.h
PythonWrapper.o: PythonWrapper.cpp PythonWrapper.h

clean:
	rm -f WebServer *.o

