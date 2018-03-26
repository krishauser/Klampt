# Klamp't Tutorial: Installation on Mac OSX

In this tutorial we learn how to install Klamp't, step by step.

Difficulty: easy

Time: 10-30 minutes


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
