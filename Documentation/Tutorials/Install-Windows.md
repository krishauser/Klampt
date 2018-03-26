# Klamp't Tutorial: Installation on Windows

In this tutorial we learn how to install Klamp't, step by step.

Difficulty: easy

Time: 10-30 minutes

- [Windows, from binaries](#windows-from-binaries)
- [Windows, from source](#windows-from-source)



# Windows, from binaries (recommended)

To run apps and obtain the C++ include/static library files:

1. First, install the *x86 version* of the [Visual Studio 2015 Runtime](https://www.microsoft.com/en-us/download/details.aspx?id=48145).
2. Run the [Klamp't installer](http://motion.pratt.duke.edu/software/Klampt-0.7.0-win32.msi). (Users of Visual Studio 2012 who want to compile their own apps should replace the lib file with [this version](http://motion.pratt.duke.edu/software/Klampt_lib-0.7.0.win32-vs2012.zip))
3. Optionally, install [ffmpeg from this binary build](http://ffmpeg.zeranoe.com/builds/win32/static/ffmpeg-20140609-git-6d40849-win32-static.7z), which will help you compile animations into MPEG movies.

To install the Klamp't Python bindings:

1. Install Python 2.7.x from [http://www.python.org/getit/](http://www.python.org/getit/). *Make sure to get the Win32 version even if you have a 64-bit machine.*
2. Add C:\Python27 to your PATH environment variable. (Right click My Computer -> Properties -> Advanced System Settings -> Environment Variables and append ";C:\Python27" to the PATH variable.)
3. Install PyOpenGL from https://pypi.python.org/pypi/PyOpenGL/ using the Win32 installer.
4. Download and install the [Win32 Klamp't Python 2.7 bindings](http://motion.pratt.duke.edu/software/Klampt-0.7.0.win32-py2.7.exe).

You're done! As a test, run "cmd" from the start menu, change directories to [Klampt install path]/Python/demos, and run
```python gltemplate.py ../../data/athlete_fractal_1.xml.```

Note: if you see errors that read "glut32.dll not found", you should copy the glut32.dll file from Klampt/bin into your SysWOW64 directory (if your machine is 64-bit, i.e., most newer machines) or System32 directory (for older 32-bit machines).

Note: when using the Python API, if you see an error that reads "NoneType not callable", then your version of PyOpenGL was built with freeglut, but its DLL was not shipped with the PyOpenGL distributable. To install the DLL, grab the binaries from the [freeglut downloads page](http://www.transmissionzero.co.uk/software/freeglut-devel/) and install the 32-bit version into the Windows\SysWOW64 folder, and the 64-bit version into your Windows\System folder.

# Windows, from source

1. Install [CMake](http://www.cmake.org/).
2. Visit https://github.com/krishauser/Klampt and click "Clone on Desktop". Follow the on-screen instructions to download Git, if necesary, and clone the Klamp't Git repository.
3. Following the same steps as above, clone the KrisLibrary Git repository from https://github.com/krishauser/KrisLibrary to the Klampt/Library folder as the target location.
4. From http://klampt.org, download the appropriate Win32 Klamp't dependencies for your Visual Studio version (both Release and Debug are recommended). Unpack into Klampt/Library.
5. Run cmake-gui in your Klamp't folder. Correct any paths that cannot be found, and generate the Visual Studio project files.
    [Note: you may need to set the cmake variable BOOST_ROOT to reflect your Boost installation path using the command line option "-DBOOST_ROOT=/path/to/boost" or by setting BOOST_ROOT in cmake-gui. You may also need to set the library paths directly by checking the "Advanced" button.]
6. Build the "Klampt", "apps", and "examples" projects in Visual Studio.
7. To build the Python bindings, you must copy the [Windows Python setup.py](http://motion.pratt.duke.edu/klampt/setup.py) into your Klampt/Python folder and edit it to reflect the paths in your distribution. Then run the Visual Studio Command prompt as an administrator and navigate to Klampt/Python. If you are using VS 2012, you must first enter "set VS90COMNTOOLS=%VS110COMNTOOLS%" and with VS 2015 you must first enter "set VS90COMNTOOLS=%VS140COMNTOOLS%". Then, run "python setup.py install".
