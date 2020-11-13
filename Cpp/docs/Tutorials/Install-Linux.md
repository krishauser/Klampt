# Klamp't Tutorial: Installation on Linux

In this tutorial we learn how to install Klamp't, step by step.

Difficulty: easy

Time: 10-30 minutes

- [Prebuilt Python libraries](#prebuilt-python-libraries)
- [Linux, from binaries](#linux-from-binaries)
- [Linux, from source](#linux-from-source)

## Prebuilt Python libraries

For Python 2.7, 3.4, 3.5, 3.6, and 3.7, you should be able to simply run

- `pip install klampt`

To run a visualization, you will need PyOpenGL and PyQt5, as well as some example programs:
- `pip install PyOpenGL`
- `pip install PyQt5`
- `git clone http://github.com/krishauser/Klampt-examples` (this is needed to run example programs)
- `cd Klampt-examples/Python/demos`
- `python gl_vis.py`

If your system has both Python 2 and Python 3 installed, the Python 3 versions are preferable.  You may need to replace the commands above with `pip3` and `python3` instead.

## Linux, from binaries

Compiled binary packages are not available for Klampt 0.8.0+.


## Linux, from source (recommended)

The following commands will install Klamp't onto your system from source.  This will ensure that you have the latest updates.

1. Make sure you have CMake, GLPK, and Qt5 (or Qt4) on your system. On systems with apt-get, the following command will do the trick:

    ```
    sudo apt-get install g++ cmake git libglpk-dev python3-dev libxmu-dev libxi-dev qt5-default
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
    cd Klampt/Cpp/Dependencies; make unpack-deps; make deps
    ```

    If this step fails, some system-dependent configuration may need to be performed. Please consult the Installation section of the [Klamp't manual](../Manual-Installation.md) for help on resolving these issues.

    (A hint: if you have a multiprocessor, KrisLibrary can be built faster by running "make -j 8" where you can replace 8 with your total number of cores. Just modify the corresponding line in the Makefile under dep-KrisLibrary to do so.)
5. Configure Klamp't via CMake:

    ```
    cd ../../
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

You're done! 

Now download the examples and try running the SimTest app
```
git clone http://github.com/krishauser/Klampt-examples
bin/SimTest Klampt-examples/data/athlete_fractal_1.xml
```

Or run a simulation from the Python API:

```
cd Klampt-examples/Python/demos
python kbdrive.py ../../data/tx90roll.xml
```

Make sure to `pip install PyOpenGL` and `pip install PyQt5`.

