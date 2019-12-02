# Klampt-jupyter-extension

This folder contains the [Jupyter notebook](http://jupyter.org) extension for [Klamp't](https://github.com/krishauser/Klampt).

*If you installed Klamp't Python API from pip*, you may prefer to install from [the standalone Jupyter extension project](https://github.com/krishauser/Klampt-jupyter-extension) because it is much lighter weight than downloading the full Klamp't source.

## Installation

To make any use of this extension, you will need to install:

- [Jupyter notebook](http://jupyter.org) 
- Klamp't 0.8.x Python API: try `pip install klampt`, or follow the [installation instructions here](http://motion.pratt.duke.edu/klampt/pyklampt_docs/Manual-Installation.html).

### Linux / OSX

To install the extension, you can simply enter

`sudo make install`

in this directory.

For more control, you can use the following Makefile targets:

- `install-user`: Installs this in the Jupyter user prefix
- `install-wurlitzer`: The wurlitzer library for capturing C++ output in Python.  This is not strictly needed, but is nice to have

### Windows

To install for all users, type the following into a command prompt:

```
jupyter nbextension install klampt/
jupyter nbextension enable klampt/main
jupyter nbextension enable klampt/three.min
jupyter nbextension enable klampt/KlamptFrontend
```

To install for just a single user, append `--user` to the first line, as follows:

```
jupyter nbextension install klampt/ --user
```

## Usage

The best way to learn how to use this extension is to study the example notebooks in the Jupyter subdirectory of the [Klamp't examples](https://github.com/krishauser/Klampt-examples) package.
Run:

> git clone https://github.com/krishauser/Klampt-examples

And then launch Jupyter:

> cd Klampt-examples/Jupyter

> jupyter notebook

The Jupyter frontend widget can be created by a `KlamptWidget`, which is defined in the `klampt.vis.ipython` module. The module also contains useful helper widgets for editing robot configurations and points.

[API documentation can be found found here](http://motion.cs.illinois.edu/klampt/pyklampt_docs/klampt.vis.ipython.html).

## Version history

0.2.0: matching to Klampt 0.8.0 release (12/31/2018)
 - Python interface is now in Klampt, not this module.
 - Widget state can be saved a bit more reliably.
 - API is closer to the klampt.vis module.

0.1.0: first release

## Wish list

- Saving standalone animations in HTML.
- Camera resetting in the GUI.
- More control over background color, lighting, textures.
- Live editing in the visualization.

## Contact

Author: Kris Hauser

University of Illinois at Urbana-Champaign

kkhauser@illinois.edu
