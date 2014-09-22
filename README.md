DriveWPACT is a set of models to size wind turbine components from the hub system, drivetrain and overall nacelle.  It is based on a collection of models beginning with a sizing model set from the University of Sunderland which was extended by the Wind Parntnerships for Advanced Component Technology (WindPACT) and later model developments at NREL as part of the NREL Cost and Scaling Model for wind plant cost analysis.

Author: [K. Dykes](mailto:katherine.dykes@nrel.gov)

## Detailed Documentation

For detailed documentation see <http://wisdem.github.io/DriveWPACT/>

## Prerequisites

NumPy, SciPy, FUSED-Wind, OpenMDAO

## Installation

Install DriveWPACT within an activated OpenMDAO environment

	$ plugin install

It is not recommended to install the software outside of OpenMDAO.

## Run Unit Tests

To check if installation was successful try to import the module

	$ python
	> import drivewpact.drive
	> import drivewpact.hub

You may also run the unit tests.

	$ python src/test/test_DriveWPACT_gradients.py

