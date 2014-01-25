
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

        check_gradient_unit_test(self, hub,display=True)

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
        hub_system.hub_mass = 10000.
        hub_system.pitch_mass = 10000.
        hub_system.spinner_mass = 1000.
        hub_system.hub_cm = [-2.,0., 1.0]
        hub_system.pitch_cm = [-2., 0., 1.0]
        hub_system.spinner_cm = [-2., 0., 1.0]
        hub_system.hub_I = [1000., 1000., 1000.]
        hub_system.pitch_I = [1000., 1000., 1000.]
        hub_system.spinner_I = [1000., 1000., 1000.]

        check_gradient_unit_test(self, hub_system,display=False)


# Nacelle Components

class Test_LowSpeedShaft(unittest.TestCase):

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

        check_gradient_unit_test(self, lss,display=False)

class Test_MainBearing(unittest.TestCase):

    def test1(self):

        mb = MainBearing()

        mb.lss_design_torque = 5000000.
        mb.lss_diameter = 1.5
        mb.lss_mass = 10000.
        mb.rotor_speed = 21.8
        mb.rotor_diameter = 126.0

        check_gradient_unit_test(self, mb,display=False)

class Test_SecondBearing(unittest.TestCase):

    def test1(self):

        sb = SecondBearing()

        sb.lss_design_torque = 5000000.
        sb.lss_diameter = 1.5
        sb.lss_mass = 10000.
        sb.rotor_speed = 21.8
        sb.rotor_diameter = 126.0

        check_gradient_unit_test(self, sb,display=False)

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

        check_gradient_unit_test(self, gbx,display=False)

class Test_HighSpeedSide(unittest.TestCase):

    def test1(self):

        hss = HighSpeedSide()

        hss.gear_ratio = 87.965

        airdensity = 1.225 # air density [kg / m^3]
        MaxTipSpeed = 80. # max tip speed [m/s]
        hss.rotor_diameter = 70. # rotor diameter [m]
        hss.rotor_speed = 21.830
        DrivetrainEfficiency = 0.95
        hss.machine_rating = 1500.
        hss.rotor_torque = (hss.machine_rating * 1000. / DrivetrainEfficiency) / (hss.rotor_speed * (np.pi / 30.)) 
            # rotor torque [Nm] calculated from max / rated rotor speed and machine rating
        
        hss.lss_diameter = 1.5

        check_gradient_unit_test(self, hss,display=False)

class Test_Generator(unittest.TestCase):

    def test1(self):

        gen = Generator()

        gen.gear_ratio = 87.965
        gen.rotor_diameter = 70. # rotor diameter [m]
        gen.machine_rating = 1500.
        gen.drivetrain_design = 1

        check_gradient_unit_test(self, gen,display=False)

class Test_Bedplate(unittest.TestCase):

    def test1(self):

        bpl = Bedplate()

        bpl.drivetrain_design = 1 # geared 3-stage Gearbox with induction generator machine

        airdensity = 1.225 # air density [kg / m^3]
        MaxTipSpeed = 80. # max tip speed [m/s]
        bpl.rotor_diameter = 70. # rotor diameter [m]
        bpl.rotor_speed = 21.830
        DrivetrainEfficiency = 0.95
        bpl.machine_rating = 1500.
        bpl.rotor_torque = (bpl.machine_rating * 1000. / DrivetrainEfficiency) / (bpl.rotor_speed * (np.pi / 30.)) 
            # rotor torque [Nm] calculated from max / rated rotor speed and machine rating
        bpl.rotor_thrust = 324000. 
        bpl.rotor_mass = 28560. # rotor mass [kg]
        bpl.tower_top_diameter = 2.7 # tower top diameter [m]

        check_gradient_unit_test(self, bpl,display=False)

class Test_AboveYawMassAdder(unittest.TestCase):

    def test1(self):

        yawadder = AboveYawMassAdder()

        yawadder.machine_rating = 1500.
        yawadder.lss_mass = 8000.
        yawadder.main_bearing_mass = 2000.
        yawadder.second_bearing_mass = 2000.
        yawadder.gearbox_mass = 60000.
        yawadder.hss_mass = 1000.
        yawadder.generator_mass = 20000.
        yawadder.bedplate_mass = 80000.
        yawadder.bedplate_length = 5.
        yawadder.bedplate_width = 3.
        yawadder.crane = True

        check_gradient_unit_test(self, yawadder,display=False)

class Test_YawSystem(unittest.TestCase):

    def test1(self):

        yaw = YawSystem()

        yaw.rotor_diameter = 70. # rotor diameter [m]
        yaw.rotor_thrust = 324000. 
        yaw.tower_top_diameter = 2.7 # tower top diameter [m]
        yaw.above_yaw_mass = 240000.

        check_gradient_unit_test(self, yaw,display=False)

class Test_NacelleSystemAdder(unittest.TestCase):

    def test1(self):

        nac = NacelleSystemAdder()

        nac.above_yaw_mass = 5000.
        nac.yawMass = 5000.
        nac.lss_mass = 5000.
        nac.main_bearing_mass = 5000.
        nac.second_bearing_mass = 5000.
        nac.gearbox_mass = 5000.
        nac.hss_mass = 5000.
        nac.generator_mass = 5000.
        nac.bedplate_mass = 5000.
        nac.mainframe_mass = 5000.
        nac.lss_cm = np.array([1.0, 1.0, 1.0])
        nac.main_bearing_cm = np.array([1.0, 1.0, 1.0])
        nac.second_bearing_cm = np.array([1.0, 1.0, 1.0])
        nac.gearbox_cm = np.array([1.0, 1.0, 1.0])
        nac.hss_cm = np.array([1.0, 1.0, 1.0])
        nac.generator_cm = np.array([1.0, 1.0, 1.0])
        nac.bedplate_cm = np.array([1.0, 1.0, 1.0])
        nac.lss_I = np.array([1.0, 1.0, 1.0])
        nac.main_bearing_I = np.array([1.0, 1.0, 1.0])
        nac.second_bearing_I = np.array([1.0, 1.0, 1.0])
        nac.gearbox_I = np.array([1.0, 1.0, 1.0])
        nac.hss_I = np.array([1.0, 1.0, 1.0])
        nac.generator_I = np.array([1.0, 1.0, 1.0])
        nac.bedplate_I = np.array([1.0, 1.0, 1.0])

        check_gradient_unit_test(self, nac,display=False)


if __name__ == "__main__":
    unittest.main()
    
