NacelleSE is a semi-empirical sizing model for a wind turbine hub and drivetrain based on WindPACT, the NREL Cost and Scaling, and University of Sunderland Cost Models.

Author: [K. Dykes](mailto:katherine.dykes@nrel.gov)

## Prerequisites

NumPy, SciPy, FUSED-Wind, OpenMDAO

## Installation

Install NacelleSE within an activated OpenMDAO environment

	$ plugin install

It is not recommended to install the software outside of OpenMDAO.

## Run Unit Tests

To check if installation was successful try to import the module

	$ python
	> import nacellese.nacellesse

You may also run the unit tests.

	$ python src/test/test_NacelleSE_gradients.py

## Detailed Documentation

Online documentation is available at <http://wisdem.github.io/NacelleSE/>