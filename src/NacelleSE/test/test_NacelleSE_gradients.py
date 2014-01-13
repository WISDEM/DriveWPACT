
#!/usr/bin/env python
# encoding: utf-8
"""
test_Turbine_CostsSE.py

Created by Katherine Dykes on 2014-01-07.
Copyright (c) NREL. All rights reserved.
"""

import unittest
import numpy as np
from commonse.utilities import check_gradient_unit_test

from NacelleSE.HubSE_components import Hub, PitchSystem, Spinner, HubSystemAdder
from NacelleSE.NacelleSE_components import LowSpeedShaft, MainBearing, SecondBearing, Gearbox, HighSpeedSide, Generator, Bedplate, AboveYawMassAdder, YawSystem, NacelleSystemAdder

# Hub Components
class Test_Hub(unittest.TestCase):

    def test1(self):

        hub = Hub()

        hub.rotor_diameter = 126.0 # m
        hub.blade_number  = 3
        hub.hub_diameter   = 3.542
        hub.rotor_bending_moment = 16665000.0 # y-direction

        check_gradient_unit_test(self, hub,display=False)

class Test_PitchSystem(unittest.TestCase):

    def test1(self):

        pitch = PitchSystem()

        pitch.blade_mass = 17740.0 # kg
        pitch.rotor_diameter = 126.0 # m
        pitch.blade_number  = 3
        pitch.hub_diameter   = 3.542
        pitch.rotor_bending_moment = 16665000.0 # y-direction

        check_gradient_unit_test(self, pitch,display=False)

class Test_Spinner(unittest.TestCase):

    def test1(self):

        spinner = Spinner()

        spinner.rotor_diameter = 126.0 # m
        spinner.hub_diameter   = 3.542

        check_gradient_unit_test(self, spinner,display=False)

class Test_HubSystemAdder(unittest.TestCase):

    def test1(self):

        hub_system = HubSystemAdder()
        hub_system.hubMass = 10000.
        hub_system.pitchMass = 10000.
        hub_system.spinnerMass = 1000.
        hub_system.hubCM = [-2.,0., 1.0]
        hub_system.pitchCM = [-2., 0., 1.0]
        hub_system.spinnerCM = [-2., 0., 1.0]
        hub_system.hubI = [1000., 1000., 1000.]
        hub_system.pitchI = [1000., 1000., 1000.]
        hub_system.spinnerI = [1000., 1000., 1000.]

        check_gradient_unit_test(self, hub_system,display=False)


        '''lss.drivetrain_design = 1 # geared 3-stage Gearbox with induction generator machine
        lss.machine_rating = 1500 # machine rating [kW]
        lss.gear_ratio = 87.965
        lss.gear_configuration = 'epp'
        lss.bevel = 0
        lss.crane = True

        airdensity = 1.225 # air density [kg / m^3]
        MaxTipSpeed = 80. # max tip speed [m/s]
        lss.rotor_diameter = 70. # rotor diameter [m]
        lss.rotor_speed = 21.830
        DrivetrainEfficiency = 0.95
        lss.rotor_torque = (lss.machine_rating * 1000. / DrivetrainEfficiency) / (lss.rotor_speed * (np.pi / 30.)) 
            # rotor torque [Nm] calculated from max / rated rotor speed and machine rating
        lss.rotor_thrust = 324000. 
        lss.rotor_mass = 28560. # rotor mass [kg]

        lss.tower_top_diameter = 2.7 # tower top diameter [m]'''

# Nacelle Components

'''class Test_LowSpeedShaft(unittest.TestCase):

    def test1(self):

        lss = LowSpeedShaft()

        airdensity = 1.225 # air density [kg / m^3]
        MaxTipSpeed = 80. # max tip speed [m/s]
        lss.rotor_diameter = 70. # rotor diameter [m]
        lss.rotor_speed = 21.830
        DrivetrainEfficiency = 0.95
        lss.machine_rating = 5000.0
        lss.rotor_torque = (lss.machine_rating * 1000. / DrivetrainEfficiency) / (lss.rotor_speed * (np.pi / 30.)) 
            # rotor torque [Nm] calculated from max / rated rotor speed and machine rating
        lss.rotor_mass = 28560. # rotor mass [kg]

        check_gradient_unit_test(self, lss,display=False)'''

'''class Test_MainBearing(unittest.TestCase):

    def test1(self):

        mb = MainBearing()

        mb.lss_design_torque = 5000000.
        mb.lss_diameter = 1.5
        mb.lss_mass = 10000.
        mb.rotor_speed = 21.8
        mb.rotor_diameter = 126.0

        check_gradient_unit_test(self, mb,display=True)

class Test_SecondBearing(unittest.TestCase):

    def test1(self):

        sb = SecondBearing()

        check_gradient_unit_test(self, sb,display=True)'''

class Test_Gearbox(unittest.TestCase):

    def test1(self):

        gbx = Gearbox()
        
        gbx.drivetrain_design = 1 # geared 3-stage Gearbox with induction generator machine
        gbx.machine_rating = 1500 # machine rating [kW]
        gbx.gear_ratio = 87.965
        gbx.gear_configuration = 'epp'
        gbx.bevel = 0

        airdensity = 1.225 # air density [kg / m^3]
        MaxTipSpeed = 80. # max tip speed [m/s]
        gbx.rotor_diameter = 70. # rotor diameter [m]
        gbx.rotor_speed = 21.830
        DrivetrainEfficiency = 0.95
        gbx.rotor_torque = (gbx.machine_rating * 1000. / DrivetrainEfficiency) / (gbx.rotor_speed * (np.pi / 30.)) 

        check_gradient_unit_test(self, gbx,display=True)

'''class Test_HighSpeedSide(unittest.TestCase):

    def test1(self):

        hss = HighSpeedSide()

        check_gradient_unit_test(self, hss,display=True)

class Test_Generator(unittest.TestCase):

    def test1(self):

        gen = Generator()

        check_gradient_unit_test(self, gen,display=True)

class Test_Bedplate(unittest.TestCase):

    def test1(self):

        bpl = Bedplate()

        check_gradient_unit_test(self, bpl,display=True)

class Test_AboveYawMassAdder(unittest.TestCase):

    def test1(self):

        yawadder = AboveYawMassAdder()

        check_gradient_unit_test(self, yawadder,display=True)

class Test_YawSystem(unittest.TestCase):

    def test1(self):

        yaw = YawSystem()

        check_gradient_unit_test(self, yaw,display=True)

class Test_NacelleSystemMassAdder(unittest.TestCase):

    def test1(self):

        nac = NacelleSystemMassAdder

        check_gradient_unit_test(self, nac,display=True)'''


if __name__ == "__main__":
    unittest.main()
    
