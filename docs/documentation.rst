.. _documentation-label:

.. currentmodule:: drivewpact.drive

Documentation
--------------

.. only:: latex

    An HTML version of this documentation is available which is better formatted for reading the code documentation and contains hyperlinks to the source code.


Turbine component sizing models for hub and drivetrain components are described along with mass-cost models for the full set of turbine components from the rotor to tower and foundation.

Documentation for DriveWPACT
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The following inputs and outputs are defined for DriveWPACT:

.. literalinclude:: ../src/drivewpact/drive.py
    :language: python
    :start-after: DriveWPACT(Assembly)
    :end-before: def configure(self)
    :prepend: class DriveWPACT(Assembly):

Implemented Base Model
=========================
.. module:: drivewpact.drive
.. class:: NacelleBase

Referenced Sub-System Modules 
==============================
.. module:: drivewpact.drive
.. class:: LowSpeedShaft
.. class:: MainBearing
.. class:: SecondBearing
.. class:: Gearbox
.. class:: HighSpeedSide
.. class:: Generator
.. class:: Bedplate
.. class:: AboveYawMassAdder
.. class:: YawSystem
.. class:: NacelleSystemAdder


.. currentmodule:: drivewpact.hub

Documentation for HubWPACT
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The following inputs and outputs are defined for HubWPACT:

.. literalinclude:: ../src/drivewpact/hub.py
    :language: python
    :start-after: HubWPACT(Assembly)
    :end-before: def configure(self)
    :prepend: class HubWPACT(Assembly):

Implemented Base Model
=========================
.. module:: drivewpact.hub
.. class:: HubBase

Referenced Sub-System Modules 
==============================
.. module:: drivewpact.hub
.. class:: Hub
.. class:: PitchSystem
.. class:: Spinner
.. class:: HubSystemAdder
