# DriveWPACT
## WARNING- This repository is deprecated

DriveWPACT is a set of models to size wind turbine components from the hub system, drivetrain and overall nacelle.  It is based on a collection of models beginning with a sizing model set from the University of Sunderland which was extended by the Wind Parntnerships for Advanced Component Technology (WindPACT) and later model developments at NREL as part of the NREL Cost and Scaling Model for wind plant cost analysis.

Author: [NREL WISDEM Team](mailto:systems.engineering@nrel.gov) 

## Documentation

See local documentation in the `docs`-directory or access the online version at <http://wisdem.github.io/DriveWPACT/>

## Installation

For detailed installation instructions of WISDEM modules see <https://github.com/WISDEM/WISDEM> or to install DriveWPACT by itself do:

    $ python setup.py install

## Run Unit Tests

To check if installation was successful try to import the package:

	$ python
	> import drivewpact.drive
	> import drivewpact.hub

You may also run the unit tests which include functional and gradient tests.  Analytic gradients are provided for variables only so warnings will appear for missing gradients on model input parameters; these can be ignored.

	$ python src/test/test_DriveWPACT.py

For software issues please use <https://github.com/WISDEM/DriveWPACT/issues>.  For functionality and theory related questions and comments please use the NWTC forum for [Systems Engineering Software Questions](https://wind.nrel.gov/forum/wind/viewtopic.php?f=34&t=1002).
