Installation
------------

.. admonition:: prerequisites
   :class: warning

   NumPy, SciPy, FUSED-Wind, OpenMDAO

Clone the repository at `<https://github.com/WISDEM/DriveWPACT>`_
or download the releases and uncompress/unpack (DriveWPACT.py-|release|.tar.gz or DriveWPACT.py-|release|.zip)

To install DriveWPACT, first activate the OpenMDAO environment and then install with the following command.

.. code-block:: bash

   $ plugin install

To check if installation was successful try to import the module

.. code-block:: bash

    $ python

.. code-block:: python

    > import drivewpact.drive
    > import drivewpact.hub

or run the unit tests for the gradient checks

.. code-block:: bash

   $ python src/test/test_DriveWPACT_gradients.py

An "OK" signifies that all the tests passed.

.. only:: latex

    An HTML version of this documentation that contains further details and links to the source code is available at `<http://wisdem.github.io/DriveWPACT>`_