DriveWPACT is a set of models to size wind turbine components from the hub system, drivetrain and overall nacelle.  It is based on a collection of models beginning with a sizing model set from the University of Sunderland which was extended by the Wind Parntnerships for Advanced Component Technology (WindPACT) and later model developments at NREL as part of the NREL Cost and Scaling Model for wind plant cost analysis.

Author: [K. Dykes](mailto:nrel.wisdem+drivewpact@gmail.com)

## Version

This software is a beta version 0.1.0.

## Detailed Documentation

For detailed documentation see <http://wisdem.github.io/DriveWPACT/>

## Prerequisites

General: NumPy, SciPy, Swig, pyWin32, MatlPlotLib, Lxml, OpenMDAO

## Dependencies

Wind Plant Framework: [FUSED-Wind](http://fusedwind.org) (Framework for Unified Systems Engineering and Design of Wind Plants)

Sub-Models: CommonSE

Supporting python packages: Pandas, Algopy, Zope.interface, Sphinx, Xlrd, PyOpt, py2exe, Pyzmq, Sphinxcontrib-bibtex, Sphinxcontrib-zopeext, Numpydoc, Ipython

## Installation

First, clone the [repository](https://github.com/WISDEM/DriveWPACT)
or download the releases and uncompress/unpack (DriveWPACT.py-|release|.tar.gz or DriveWPACT.py-|release|.zip) from the website link at the bottom the [DriveWPACT site](http://nwtc.nrel.gov/DriveWPACT).

Install DriveWPACT within an activated OpenMDAO environment:

	$ plugin install

It is not recommended to install the software outside of OpenMDAO.

## Run Unit Tests

To check if installation was successful try to import the module from within an activated OpenMDAO environment:

	$ python
	> import drivewpact.drive
	> import drivewpact.hub

You may also run the unit tests which include functional and gradient tests.  Analytic gradients are provided for variables only so warnings will appear for missing gradients on model input parameters; these can be ignored.

	$ python src/test/test_DriveWPACT.py

For software issues please use <https://github.com/WISDEM/DriveWPACT/issues>.  For functionality and theory related questions and comments please use the NWTC forum for [Systems Engineering Software Questions](https://wind.nrel.gov/forum/wind/viewtopic.php?f=34&t=1002).