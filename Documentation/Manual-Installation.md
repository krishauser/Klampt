# Klamp't Manual: Downloading and building Klamp't

## Python-only install

If you only need the Python API, you can install using pip.  Simply open up a command line window and call

> pip install klampt

As of writing, pip packages are available for Linux (Python 2.7, 3.5, 3.6, 3.7), Windows (Python 2.7 and 3.7, 32- and 64-bit), and Mac OSX (Python 2.7 and 3.7).

You should also obtain:
- PyOpenGL for visualization.  Try `pip install PyOpenGL` ( [https://pypi.python.org/pypi/PyOpenGL/3.0.2](https://pypi.python.org/pypi/PyOpenGL/3.0.2)).
- PyQt5 is highly recommended for resource editing and improved visualization.  Try `pip install PyQt5`. PyQt4 is also supported for now, but at some point the package will be deprecated.
- Python Imaging Library (PIL) is required for saving screenshots to disk.

Python 2.7 Windows users will want to install the [correct binaries from here](https://www.lfd.uci.edu/~gohlke/pythonlibs/#pyqt4)

You will also want to get the Klampt-examples repository to test your install, e.g.,:

```sh
git clone https://github.com/krishauser/Klampt-examples
cd Klampt-examples/Python/demos
python kbdrive.py ../../data/tx90roll.xml
```


## Building from scratch, and C++ builds

Klamp't is publicly available via the git repository at [https://github.com/krishauser/Klampt/](https://github.com/krishauser/Klampt/). The command
```sh
git clone https://github.com/krishauser/Klampt
```
will download the required files.

You will also need to obtain the following dependencies, which may already be installed on your machine:

- CMake (version &gt;= 2.6)
- GLUT
- GLPK, the GNU Linear Programming Kit
- Python, if you wish to use the Python bindings (compatible with Python 2.6, 2.7 and 3.x).
- (recommended) Assimp, if you wish to load STL, DAE and other geometry file formats.  (Only OBJ and OFF are natively supported in Klampt.)
- (recommended) Qt5, if you wish to use nicer GUIs for the core applications.

## Linux-like Environments

**Building dependencies.** First, the dependencies must be downloaded and built. GLUT and GLPK must first be installed in your library paths. Change into the Klampt/Library folder and unpack KrisLibrary, TinyXML, GLUI, and ODE using the command 'make unpack-deps'. After configuring the dependencies as described below, they can be built using the command 'make deps'.

To configure the dependencies, consider the following notes:

- KrisLibrary may need to be configured for your particular system. Try running cmake-gui and changing the Advanced variables.
- By default, we compile ODE in double floating-point precision.  The reason for this is that on some Linux systems, ODE becomes unstable in single floating-point precision and may crash with assertion failures. This may be changed on other systems, if you wish, by toggling ODEDOUBLE=0 or 1 in Klampt/Library/Makefile. _Note: if you have already built ODE and then later change its precision, you must do a clean build of ODE as well as the CMake cache._

**Enabling Assimp support (optional).** To load a larger variety of 3D meshes, Klamp't can be configured to use the Asset Importer ( [Assimp](http://assimp.sourceforge.net/)) library. Once Assimp 3.0.1270 is installed on your system (if Klampt/Library/assimp--3.0.1270-sdk or /usr/lib/libassimp.so exists), KrisLibrary and Klampt should automatically detect it when built.

**Run CMake to build Klamp't Makefiles.** Run &quot;cmake .&quot; to build the Klamp't makefiles.

**Building static library and apps.** The static library is built using 'make Klampt'. The main apps to build are RobotTest, SimTest, and RobotPose. Typing 'make [target]' will build the target.

**Building Python bindings.** Once the Klamp't static library is built, the Python bindings in Klampt/Python/klampt can be built using &quot;make python&quot;. To install the klampt module into your Python package, type &quot;make python-install&quot;.

IMPORTANT: You must set up Python to be able to find the shared library files for external dependencies. Otherwise, you will get errors importing the \_robotsim module when calling import klampt. To do this, you may either:

1. Set the LD\_LIBRARY\_PATH environment variable to include the locations of the TinyXML, ODE, and (optionally) Assimp shared libraries.  These will be .so (or DLL) files.
2. OR move the shared library files into your shared library path
3. OR on Linux-like systems, edit /etc/ld.so.conf as appropriate and then run ldconfig (as sudo).

**Platform-specific install scripts**

These commands work from a clean install of Ubuntu 12.04

```sh
sudo apt-get freeglut3 freeglut3-dev glpk python-opengl
```

[Optional: to enable Assimp mesh importing, before calling any of the "make";" calls, call `sudo apt-get install libassimp-dev`]

```sh
cd Klampt
cd Cpp/Dependencies
make unpack-deps
make deps
cd ../../
cmake .
make all
sudo make python-install
```

**Building documentation.** To build the Klamp't C++ API documentation using Doxygen, type `make docs` in Klampt/. `make python-docs` will build the Python API documentation.

## Windows

Prebuilt binary executables and static libraries for VS2015 are available on the Klamp't website. 

**Step by step instructions to install the C++ applications from binaries**

From [http://klampt.org](http://klampt.org/), download and run the Win32 Klamp't installer.  Note: If you plan to develop in the Klamp't C++ API, make sure to get the appropriate installer for your Visual Studio version.

**Step by step instructions to install the Python API from binaries**

Follow the instructions at the top of this file.


**To build your own C++ applications that link to Klamp't**

1. Follow the instructions to install the C++ applications from binaries.
2. Clone the KrisLibrary Git repository from [https://github.com/krishauser/KrisLibrary](https://github.com/krishauser/KrisLibrary) to the Klampt/Library folder as the target location.
3. From [http://klampt.org](http://klampt.org/), download the appropriate Win32 Klamp't dependencies for your Visual Studio version (both Release and Debug are recommended). Unpack into Klampt/Library.
4. In your own CMake project, set `KLAMPT_ROOT` to the appropriate path and put the following lines into your CMakeLists.txt (along with whatever other lines are needed to build your project)
```cmake
SET (CMAKE_MODULE_PATH "${KLAMPT_ROOT}/CMakeModules")
FIND_PACKAGE(Klampt REQUIRED)
ADD_DEFINITIONS(${KLAMPT_DEFINITIONS})
INCLUDE_DIRECTORIES(${KLAMPT_INCLUDE_DIRS})
TARGET_LINK_LIBRARIES(MyApp ${KLAMPT_LIBRARIES})
```
5. Build your project in standard CMake fashion.

**Building Klamp't from source.** After following the instructions under the heading &quot;To build your own C++ applications that link to Klamp't&quot;, the standard CMake procedure should generate appropriate Visual Studio project files.

**Building Python bindings from source.** (tested with Python 2.7, Win32) The standard CMake procedure should generate Visual Studio project files for the project &quot;python-install&quot; but these are broken. Instead, download the Windows Python setup.py file from [http://klampt.org](http://klampt.org/) and copy it to the Klampt/Python directory.  Edit the paths at the top of the file to reflect your computer's file structure.  Finally, open a Visual Studio Command Prompt in _Administrative Mode_, and depending on your VS version, run:

VS 2008: `python setup.py install`

VS 2010:
```sh
set VS90COMNTOOLS=%VS100COMNTOOLS%
python setup.py install
```

VS 2012:
```sh
set VS90COMNTOOLS=%VS110COMNTOOLS%
python setup.py install
```

VS 2015:
```sh
set VS90COMNTOOLS=%VS140COMNTOOLS%
python setup.py install
```

**Building dependencies from source.**

If you wish to build dependencies from scratch, Visual Studio project files are available. Make sure to place all compiled library (.lib) files in the Klampt/Cpp/Dependencies folder.  All libraries should be built in Win32 mode, with C++ code generation set to Multithreaded DLL / Multithreaded Debug DLL.

The general procedure is as follows:

1. Acquire GLEW and optionally (but recommended) WinGLPK 4.61 and/or Assimp 3.0.1270. Place the glew32.lib, glpk\_4\_61.lib files in Klampt/Cpp/Dependencies or in your Visual Studio path. Place the Assimp folder in Klampt/Library.
2. Configure and edit dependencies as follows:
    1. ODE: Set up build files with premake4 vs2010.
3. Compile all dependencies except for KrisLibrary. Place all generated .lib files into the Klampt/Cpp/Dependencies directory.
    1. ODE: compile in double precision, Static.
    2. TinyXML: compile with STL support.
4. Compile KrisLibrary last. CMake files are available for compiling KrisLibrary with/without Assimp support and with/without GLPK support. 
5. After compiling, all of the .dll files associated with dependency libraries should be placed in the appropriate Klamp't binary folders.

