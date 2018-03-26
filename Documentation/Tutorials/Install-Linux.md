# Klamp't Tutorial: Installation on Linux

In this tutorial we learn how to install Klamp't, step by step.

Difficulty: easy

Time: 10-30 minutes

- [Linux, from binaries](#linux-from-binaries)
- [Linux, from source](#linux-from-source)

## Linux, from binaries

Compiled binary packages are available for the following systems:

- [Linux x86, 64-bit Debian package](http://motion.pratt.duke.edu/software/Klampt-0.7.0-Linux.deb)
- Linux x86, 64-bit Klamp't Python bindings, [for Python version 2.7](http://motion.pratt.duke.edu/software/Klampt-0.7.0-python2.7.x86_64.rpm)

To compile your own programs, you will need the same dependencies as compiling from scratch. To save time compiling KrisLibrary, you can make use of the following compiled binary:
Linux x86, 64-bit Debian package for KrisLibrary dependency


## Linux, from source (recommended)

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

    If this step fails, some system-dependent configuration may need to be performed. Please consult the Installation section of the [Klamp't manual](../Manual-Installation.md) for help on resolving these issues.

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

