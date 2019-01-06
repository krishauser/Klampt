Installing Klamp't
================================================

If you only need the Python API, you can install using pip. Simply open
up a command line window and call::

    pip install klampt

As of writing, pip packages are available for Linux (Python 2.7, 3.5,
3.6, 3.7), Windows (Python 2.7, 3.5, 3.6, and 3.7, 32- and 64-bit), and
Mac OSX 10.9 and higher (Python 2.7, 3.5, 3.6, and 3.7). These are built
with Assimp (mesh loading) and GLEW (OpenGL supported rendering of
camera images). They **do not** have ROS or OMPL support, and if you
want those you will need to build from source.

You should also obtain:

-  PyOpenGL for visualization. Try ``pip install PyOpenGL``.
-  PyQt5 is highly recommended for resource editing and improved
   visualization. Try ``pip install PyQt5``. (PyQt4 is also supported
   for now, but at some point the package will be deprecated.)
-  Python Imaging Library (PIL) is required for saving screenshots to
   disk.

Klampt-examples
----------------

You will also want to get the `Klampt-examples <https://github.com/krishauser/Klampt-examples>`__ repository to test your
install, e.g.,:

.. code:: sh

    git clone https://github.com/krishauser/Klampt-examples
    cd Klampt-examples/Python/demos
    python kbdrive.py ../../data/tx90roll.xml

Most of the examples in this manual require Klampt-examples to be downloaded to your computer.


Troubleshooting
---------------

Python 2.7 Windows users will want to install the `correct PyQt4
binaries from
here <https://www.lfd.uci.edu/~gohlke/pythonlibs/#pyqt4>`__

You may get errors importing the \_robotsim module when calling
``import klampt``. This usually means some dependency DLL is missing on
your system. If this occurs, `please file an issue
report <https://github.com/krishauser/Klampt/issues>`__ and we will get
on it. If you are using Windows, please use the
`Dependencies <https://github.com/lucasg/Dependencies>`__ program to
open the ``_robotsim.pyd`` file, and report which DLLs are missing.


Jupyter notebook integration
----------------------------

The `Klampt-jupyter-extension <https://github.com/krishauser/Klampt-jupyter-extension>`__ project
is a companion project that allows you to install Klamp't visualizations into Jupyter Notebook.
Simply enter

.. code:: sh

      git pull https://github.com/krishauser/Klampt-jupyter-extension
      cd Klampt-jupyter-extension
      sudo make install

and the next time you run Jupyter notebooks, you can use the functionality of the
`klampt.vis.ipython <klampt.vis.ipython.html>`__ module to display interactive 3D displays
in your jupyter notebook!

.. image:: _static/images/jupyter.png

.. note::
    Klampt-jupyter-extension is already included in the Klampt source distribution
    under the Klampt/Jupyter folder.  If you are building from source, just enter

    .. code:: sh

          cd Klampt/Jupyter
          sudo make install


Should I build from source?
----------------------------

If you are running on Linux or Mac, please consider `building from source <Manual-BuildingSource.html>`__. 
In particular, building from source has the following advantages:

-  The RobotTest, SimTest, RobotPose, and URDFtoRob apps are extremely useful utilities.
-  The Python API can be built with ROS support to show live point clouds in Klampt.
-  You will have access to the latest updates with a simple ``git pull``.

