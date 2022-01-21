# Klamp't Manual: Downloading and building Klamp't

## Python-only install

If you only need the Python API, please consult the [Python API documentation](http://motion.cs.illinois.edu/sofware/klampt/0.8/pyklampt_docs/Manual-Installation.html) for install instructions.


## Building from source

Klamp't C++ source code is available via the git repository at [https://github.com/krishauser/Klampt/](https://github.com/krishauser/Klampt/). The command
```sh
git clone https://github.com/krishauser/Klampt
```
will download the repository.

To build the Klampt C++ library and Python API, you will need to obtain the following dependencies, which may already be installed on your machine:

- CMake (version &gt;= 2.6)
- GLPK, the GNU Linear Programming Kit
- Python, if you wish to build the Python bindings (compatible with Python 2.7 and 3.x).
- (recommended) Assimp, if you wish to load STL, DAE and other geometry file formats.  (Only OBJ and OFF are natively supported in Klampt.)
- (recommended) Qt5, if you wish to use nicer GUIs for the core applications.
- (optional) ROS, if you wish to write to/read from ROS topics.
- (optional) OMPL, if you wish to use the OMPL motion planning bindings.
- (optional) GLUT, if you wish to use the legacy GLUI programs.




## Linux-like Environments

**The [Installation Tutorial](Tutorials/Install-Linux.md) has step by step instructions, but if you get stuck, perhaps the below instructions may help.**

**Building dependencies.** First, the dependencies must be downloaded and built. GLPK must first be installed in your library path. Change into the Klampt/Cpp/Dependencies folder and unpack KrisLibrary, TinyXML, and ODE using the command 'make unpack-deps'. After configuring the dependencies as described below, they can be built using the command 'make deps'.

To configure the dependencies, consider the following notes:

- KrisLibrary may need to be configured for your particular system. Try running `cmake-gui` and changing the Advanced variables.
- By default, we compile ODE in double floating-point precision.  The reason for this is that on some Linux systems, ODE becomes unstable in single floating-point precision and may crash with assertion failures. This may be changed on other systems, if you wish, by toggling ODEDOUBLE=0 or 1 in Klampt/Cpp/Dependencies/Makefile. _Note: if you have already built ODE and then later change its precision, you must do a clean build of ODE as well as the CMake cache._

**Enabling Assimp support (optional).** To load a larger variety of 3D meshes, Klamp't can be configured to use the Asset Importer ( [Assimp](http://assimp.sourceforge.net/)) library. Once Assimp 3.0.1270 is installed on your system (if Klampt/Cpp/Depdencies/assimp--3.0.1270-sdk or /usr/lib/libassimp.so exists), KrisLibrary and Klampt should automatically detect it when built.

**Run CMake to build Klamp't Makefiles.** Run &quot;cmake .&quot; to build the Klamp't makefiles.

**Building static library and apps.** The static library is built using 'make Klampt'. The main apps to build are RobotTest, SimTest, and RobotPose. Typing 'make [target]' will build the target.

**Building Python bindings.** Once the Klamp't static library is built, the Python bindings in Klampt/Python/klampt can be built using &quot;make python&quot;. To install the klampt module into your Python package, type &quot;make python-install&quot;.

IMPORTANT: You must set up Python to be able to find the shared library files for external dependencies. Otherwise, you will get errors importing the \_robotsim module when calling import klampt. To do this, you may either:

1. Set the LD\_LIBRARY\_PATH environment variable to include the locations of the TinyXML, ODE, and (optionally) Assimp shared libraries.  These will be .so (or DLL) files.
2. OR move the shared library files into your shared library path
3. OR on Linux-like systems, edit /etc/ld.so.conf as appropriate and then run ldconfig (as sudo).

**Building documentation.** To build the Klamp't C++ API documentation using Doxygen, type `make docs` in Klampt/. `make python-docs` will build the Python API documentation.



## Windows

Prebuilt binary executables and static libraries for VS2015 are available on the [Klamp't website](http://klampt.org). 

### Installling C++ applications from binaries**

From [http://klampt.org](http://klampt.org/), download and run the Win32 Klamp't installer. 




### To build your own C++ applications that link to Klamp't

1. Clone the Klampt Git repository from [https://github.com/krishauser/Klampt](https://github.com/krishauser/Klampt)
2. Run the Win32 Klamp't installer or obtain the x64 binary package.  Copy the include and lib directories to `Klampt/`
3. Clone the KrisLibrary Git repository from [https://github.com/krishauser/KrisLibrary](https://github.com/krishauser/KrisLibrary) to the `Klampt/Cpp/Dependencies` folder as the target location.
4. From [http://klampt.org](http://klampt.org/), download the appropriate Win32 Klamp't dependencies for your Visual Studio version (both Release and Debug are recommended). Unpack into Klampt/Cpp/Dependencies.
5. In your own CMake project, set `KLAMPT_ROOT` to the appropriate path and put the following lines into your CMakeLists.txt (along with whatever other lines are needed to build your project)
```cmake
SET (CMAKE_MODULE_PATH "${KLAMPT_ROOT}/CMakeModules")
FIND_PACKAGE(Klampt REQUIRED)
ADD_DEFINITIONS(${KLAMPT_DEFINITIONS})
INCLUDE_DIRECTORIES(${KLAMPT_INCLUDE_DIRS})
TARGET_LINK_LIBRARIES(MyApp ${KLAMPT_LIBRARIES})
```
6. Build your project in standard CMake fashion.


### Building Klamp't from source

After following Steps 1, 3, and 4 in the instructions above, running the standard CMake procedure in `Klampt/` (using `Klampt/msvc` as the build location for 32-bit windows, or Klampt/msvc64 as the build location for 64-bit windows) should generate appropriate Visual Studio project files.

### Building Python bindings from source

After running CMake as above, copy the file `Klampt/msvc/Python/setup.py` (or `Klampt/msvc64/Python/setup.py`) to the `Klampt/Python` directory.  Finally, open a Visual Studio Command Prompt in _Administrative Mode_, and depending on your VS version, run:

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

### Building dependencies from source

If you wish to build dependencies from scratch, Visual Studio project files are available. Make sure to place all compiled library (.lib) files in the Klampt/Cpp/Dependencies folder.  All libraries should be built in Win32 mode, with C++ code generation set to Multithreaded DLL / Multithreaded Debug DLL.

The general procedure is as follows:

1. Acquire GLEW and optionally (but recommended) WinGLPK 4.61 and/or Assimp 3.0.1270. Place the glew32.lib, glpk\_4\_61.lib files in Klampt/Cpp/Dependencies or in your Visual Studio path. Place the Assimp folder in Klampt/Cpp/Dependencies.
2. Configure and edit dependencies as follows:
    1. ODE: Set up build files with premake4 vs2010.
3. Compile all dependencies except for KrisLibrary. Place all generated .lib files into the Klampt/Cpp/Dependencies directory.
    1. ODE: compile in double precision, Static.
    2. TinyXML: compile with STL support.
4. Compile KrisLibrary last. CMake files are available for compiling KrisLibrary with/without Assimp support and with/without GLPK support. 
5. After compiling, all of the .dll files associated with dependency libraries should be placed in the appropriate Klamp't binary folders.

