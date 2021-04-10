# Klamp't Tutorial: Installation on Windows

In this tutorial we learn how to install Klamp't, step by step.

Difficulty: easy

Time: 10-30 minutes

- [Prebuilt Python libraries](#prebuilt-python-libraries)
- [Windows, from binaries](#windows-from-binaries)
- [Windows, from source](#windows-from-source)


# Prebuilt Python libraries

To install the Klamp't Python bindings:

1. Install Python 2.7+ or 3.7+ from [http://www.python.org/getit/](http://www.python.org/getit/).  Both 32-bit (x86) and 64-bit (x64) versions are supported.
2. Install Klampt using `pip install klampt`
3. Install PyOpenGL using `pip install PyOpenGL`, or use the Win32 installer from https://pypi.python.org/pypi/PyOpenGL/.
4. Install PyQt5 or PyQt4.  In Python 3.x this is easy as `pip install PyQt5`, but users of Python 2.x will need to [download the appropriate unofficial PyQt4 build here](https://www.lfd.uci.edu/~gohlke/pythonlibs/#pyqt4).

You're done! As a test, run "cmd" from the start menu, 

```sh
git clone http://github.com/krishauser/Klampt-examples
cd Klampt-examples/Python/demos
python gl_vis.py
```

# Windows, from binaries (recommended)

To run apps and obtain the C++ include/static library files:

1. First, install the *x86 version* of the [Visual Studio 2015 Runtime](https://www.microsoft.com/en-us/download/details.aspx?id=48145).
2. Run the [Klamp't installer](http://motion.cs.illinois.edu/software/klampt/0.8/Klampt-0.8.6-win32.msi). 
3. Optionally, install [ffmpeg from this binary build](http://ffmpeg.zeranoe.com/builds/win32/static/ffmpeg-20140609-git-6d40849-win32-static.7z), which will help you compile animations into MPEG movies.


# Windows, from source

Due to the complexities of dependency management on Windows, building from source is much more challenging and not recommended unless you have extensive experience building packages on Windows via CMake.  These instructions are also bound to go out of date occasionally.

1. Install [CMake](http://www.cmake.org/).
2. Obtain Visual Studio 2015.
3. Visit https://github.com/krishauser/Klampt and click "Clone on Desktop". Follow the on-screen instructions to download Git, if necesary, and clone the Klamp't Git repository.
4. Following the same steps as above, clone the KrisLibrary Git repository from https://github.com/krishauser/KrisLibrary to the Klampt/Cpp/Dependencies folder as the target location.
5. From http://klampt.org, download the appropriate Win32 Klamp't dependencies for your Visual Studio version (both Release and Debug are recommended). Unpack into Klampt/Cpp/Dependencies.
6. Run cmake-gui in your Klamp't folder, and set the build directory to Klampt/msvc. Correct any paths that cannot be found, and generate the Visual Studio project files.
7. Open Klampt/msvc/Klampt.sln and build the "Klampt" and "apps" projects in Visual Studio.
8. To build the Python bindings:
   1. Copy Klampt/msvc/Python/setup.py into your Klampt/Python folder. 
   2. Run the Visual Studio Command prompt as an administrator and navigate to Klampt/Python. 
   3. For VS 2015 you must first enter "set VS90COMNTOOLS=%VS140COMNTOOLS%". 
   4. Run "python setup.py install".
