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
   visualization. Try ``pip install PyQt5``. (For Python 2.x users,
   PyQt4 is also supported.  See note below.)

To enable all features, you may also obtain the following optional packages:

-  PyQtGraph lets you customize PyQt visualizations through the GUI.
   Try ``pip install pyqtgraph``.
-  Python Imaging Library (PIL) is required for saving screenshots to
   disk.
-  ffmpeg is needed to save movies.
-  cvxpy is needed to use the :mod:`klampt.plan.kinetrajopt` module. 
   If you are interested in collision-free trajectory optimization, try
   ``pip install cvxpy``.

.. note::

    With Python 2.x on most platforms, PyQt5 can't be installed through pip,
    so you will need to use PyQt4 instead. (You could try building PyQt5 from
    source, but it's a tedious process.)

    Python 2.7 Windows users will want to install the `correct PyQt4 binaries from
    here <https://www.lfd.uci.edu/~gohlke/pythonlibs/#pyqt4>`__

    At some point PyQt4 will be deprecated, so eventually we will be migrating
    Klamp't to use Python 3.x by default.


Klampt-examples
----------------

You will also want to get the `Klampt-examples <https://github.com/krishauser/Klampt-examples>`__ repository to test your
install, e.g.,:

.. code:: sh

    git clone https://github.com/krishauser/Klampt-examples
    cd Klampt-examples/Python3/demos
    python kbdrive.py ../../data/tx90roll.xml

Most of the examples in this manual require Klampt-examples to be downloaded to your computer.

.. note::

    All the examples in this manual are written assuming you are using Python 3.x.  If you are using Python 2.x,
    you will need to replace any references to `Klampt-examples/Python3` with `Klampt-examples/Python2`.

    Moreover, if you have both Python 3.x and 2.x installed, but Klamp't is only installed with one version,
    then all of the calls to `python` in the examples in this manual should be replaced with calls to either
    `python3` or `python2`, whichever is appropriate.


Troubleshooting
---------------

If you have multiple versions of Python installed on your machine,
you will need to be aware of which version is currently referred to by `pip` and `python` commands.  You
may need to use `python3`, `python2`, `pip3`, or `pip2` to get the right version.  You may consider using a
`virtual environment <https://docs.python-guide.org/dev/virtualenvs/>`__ to help manage
currently active version.


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
:mod:`klampt.vis.ipython` module to display interactive 3D displays
in your jupyter notebook!

.. image:: _static/images/jupyter.png

.. note::
    Klampt-jupyter-extension is already included in the Klampt source distribution
    under the Klampt/Jupyter folder.  If you are building from source, just enter

    .. code:: sh

          cd Klampt/Jupyter
          sudo make install

To get started using Klamp't in Jupyter, browse the examples in the Jupyter directory of
the `Klampt-examples <https://github.com/krishauser/Klampt-examples>`__ repository.


Ready-to-use web interfaces
---------------------------
Klamp't works best when it is installed on your local machine, but it can also be run online through your web browser using Google Colab or Binder (or any other Jupyterhub server).

- Google Colab |colab_badge|
- Binder |binder_badge|


.. |colab_badge| image:: https://colab.research.google.com/assets/colab-badge.svg
   :target: https://colab.research.google.com/gist/krishauser/1a518571493d2582f8bda908d9db02fb/klamptcolab.ipynb
   :alt: Open in Colab

.. |binder_badge| image:: https://mybinder.org/badge_logo.svg
   :target: https://mybinder.org/v2/gh/krishauser/Klampt-examples/binder?filepath=Jupyter%2FBasicKlamptDemo.ipynb
   :alt: Open in Binder

Note that the UI functionality is drastically limited compared to a local installation.


Grabbing the latest updates
---------------------------

To grab the latest Python API updates on top of a pip install without having to
configure your environment to build from source, you can use the
``patch_a_pip_install.py`` script as follows:

.. code:: sh

    pip install --upgrade klampt
    git clone --depth 1 https://github.com/krishauser/Klampt
    cd Klampt/Python
    python patch_a_pip_install.py

Note that this will not fix any bugs in the underlying C++ API.  Please see the
`release notes <https://github.com/krishauser/Klampt#version-history>`__ to check
which updates are available in the Python API only.  


Should I build from source?
----------------------------

If you are running on Linux or Mac, please consider `building from source <Manual-BuildingSource.html>`__. 
In particular, building from source has the following advantages:

-  The RobotTest, SimTest, RobotPose, and URDFtoRob apps are extremely useful utilities.
-  The Python API can be built with ROS support to show live point clouds in Klampt.
-  You will have access to the latest updates with a simple ``git pull``.

