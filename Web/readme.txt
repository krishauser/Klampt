================================
Building / Setting Up
================================

Server:
go to /Klampt/Web/Server directory
cmake .
make

note: you may have to edit CMakeList.txt so it can properly find QT 5.5, 
see the hack where I set a path to /opt/..

Client:
make sure git has setup the submodules
go to /Klampt/Web/Client directory
git submodule init
git submodule update

================================
Running
================================

Start up the server
goto /Klampt directory
./WebServer

Start up a client
goto /Klampt/Web/Client
click on index.html
make sure server URL is correct
click connect
click submit code
click run




