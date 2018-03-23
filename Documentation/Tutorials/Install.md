# Klamp't Tutorial: Installation

In this tutorial we learn how to install Klamp't, step by step.

Difficulty: easy

Time: 10-30 minutes

[Linux, from binaries](#linux-from-binaries)
[Linux, from source](#linux-from-source)
[Windows, from binaries](#windows-from-binaries)
[Windows, from source](#windows-from-source)
[Mac OSX, from source](#mac-osx-from-source)


## Linux, from binaries

Compiled binary packages are available for the following systems:

- [Linux x86, 64-bit Debian package](http://motion.pratt.duke.edu/software/Klampt-0.7.0-Linux.deb)
- Linux x86, 64-bit Klamp't Python bindings, [for Python version 2.7](http://motion.pratt.duke.edu/software/Klampt-0.7.0-python2.7.x86_64.rpm)

To compile your own programs, you will need the same dependencies as compiling from scratch. To save time compiling KrisLibrary, you can make use of the following compiled binary:
Linux x86, 64-bit Debian package for KrisLibrary dependency


## Linux, from source

The following commands will install Klamp't onto your system from source.

1. Make sure you have CMake, GLUT, GLPK, Boost C++ libraries, and Qt4 (or Qt5) on your system. On systems with apt-get, the following command will do the trick:

    ```
    sudo apt-get install g++ cmake git libboost-system-dev libboost-thread-dev freeglut3 freeglut3-dev libglpk-dev python-dev python-opengl libxmu-dev libxi-dev libqt4-dev
    ```

2. (recommended) Download and install Assimp using the following command line:

    ```
    sudo apt-get install libassimp-dev
    ```

    or install it from source at: http://sourceforge.net/projects/assimp/files/assimp-3.0/
3. Clone the Klamp't git repository:

    ```
    git clone https://github.com/krishauser/Klampt
    ```

4. Make the Klamp't dependencies (KrisLibrary, TinyXML, ODE, GLEW, GLUI):

    ```
    cd Klampt/Library; make unpack-deps; make deps
    ```

    If this step fails, some system-dependent configuration may need to be performed. Please consult the Installation section of the [Klamp't manual](http://motion.pratt.duke.edu/klampt/KlamptManualv0.7.pdf) for help on resolving these issues.

    (A hint: if you have a multiprocessor, KrisLibrary can be built faster by running "make -j 8" where you can replace 8 with your total number of cores. Just modify the corresponding line in the Makefile under dep-KrisLibrary to do so.)
5. Configure Klamp't via CMake:

    ```
    cd ..
    cmake .
    ```

    If this step gives warnings, then some of your paths may be incorrect, and you will need to manually adjust them using the cmake-gui program. Note that if you are doing extensive development in Klamp't then you may wish to compile with debugging information instead, using the command
    ```
    cmake -DCMAKE_BUILD_TYPE=Debug .
    ```
6. Compile the Klamp't static library

    ```
    make Klampt
    ```

7. Make the apps

    ```
    make apps
    ```

8. Make and install the Klamp't Python bindings

    ```
    make python
    sudo make python-install
    ```

9. Optionally, if your system does not already have it, install [ffmpeg](http://www.ffmpeg.org) to save MPEG movies. On Ubuntu systems you may use

    ```
    sudo apt-get install ffmpeg
    ```

You're done! Try running the SimTest app:

```
./SimTest data/athlete_fractal_1.xml
```

Or run a simulation from the Python API:

```
cd Python/demos
python kbdrive.py ../../data/tx90roll.xml
```

# Windows, from binaries

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

# Mac OSX, from source

1. You will need to install the Xcode Command Line Tools. To see if they are installed, run

    ```
    xcode-select -p
    ```

    If you see

    ```
    /Applications/Xcode.app/Contents/Developer
    ```

    then the full Xcode package is already installed. If not, you'll need to run

    ```
    xcode-select --install
    ```

2. Install XQuartz from http://xquartz.macosforge.org/
3. Install homebrew from http://brew.sh
4. In the terminal, run

    ```
    brew install assimp boost cmake ffmpeg freeglut glui homebrew/science/glpk python qt5 ode --with-double-precision
    ```

5. Clone the Klamp't git repo:

    ```
    git clone https://github.com/krishauser/Klampt
    ```

6. Make the Klamp't dependencies:

    ```
    cd Klampt/Library; make unpack-deps; make deps
    ```

7. Configure Klamp't via CMake

    ```
    cd .. cmake .
    ```

    or

    ```
    cmake -DCMAKE_BUILD_TYPE=Debug .
    ```

    if you wish to have debugging information.
8. Compile the Klamp't static library:

    ```
    make Klampt
    ```

9. Make the apps:

    ```
    make apps
    ```

    and test that they work:

    ```
    ./SimTest data/athlete_fractal_1.xml
    ```

To install the Klamp't Python bindings:
1. Modify ~/.bash_profile as follows to use the homebrew version of Python:

    ```
    export PATH=~/bin:/usr/local/bin:$PATH export PYTHONPATH=/usr/local/lib/python2.7/site-packages
    ```

2. Restart your terminal window and check that you have the right Python:

    ```
    which python
    ```

    should return

    ```
    /usr/local/bin/python
    ```

3. In the terminal, install PyOpenGL with:

    ```
    pip install PyOpenGL
    ```

4. Make and install the Klamp't Python bindings:

    ```
    make python make python-install
    ```

    Note: sudo is not required for installation if you use homebrew's Python.
5. Test that the Python bindings work:

    ```
    python Python/demos/kbdrive.py data/tx90roll.xml
    ```
