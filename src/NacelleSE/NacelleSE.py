"""
nacelleSE.py

Created by Katherine Dykes 2012.
Copyright (c) NREL. All rights reserved.
"""

from openmdao.main.api import Assembly
from openmdao.main.datatypes.api import Float, Bool, Int, Str, Array
from math import pi
import numpy as np

from NacelleSE_components import LowSpeedShaft, MainBearing, SecondBearing, Gearbox, HighSpeedSide, Generator, Bedplate, AboveYawMassAdder, YawSystem, NacelleSystemAdder
from DriveSE_components import LowSpeedShaft_drive, Gearbox_drive, MainBearing_drive, SecondBearing_drive, Bedplate_drive, YawSystem_drive, LowSpeedShaft_drive3pt, LowSpeedShaft_drive4pt


class NacelleBase(Assembly):

    # variables
    rotor_diameter = Float(iotype='in', units='m', desc='rotor diameter')
    rotor_mass = Float(iotype='in', units='kg', desc='rotor mass')
    rotor_torque = Float(iotype='in', units='N*m', desc='rotor torque at rated power')
    rotor_thrust = Float(iotype='in', units='N', desc='maximum rotor thrust')
    rotor_speed = Float(iotype='in', units='rpm', desc='rotor speed at rated')
    machine_rating = Float(iotype='in', units='kW', desc='machine rating of generator')
    gear_ratio = Float(iotype='in', desc='overall gearbox ratio')
    tower_top_diameter = Float(iotype='in', units='m', desc='diameter of tower top')
    rotor_bending_moment = Float(iotype='in', units='N*m', desc='maximum aerodynamic bending moment')

    # parameters
    drivetrain_design = Int(iotype='in', desc='type of gearbox based on drivetrain type: 1 = standard 3-stage gearbox, 2 = single-stage, 3 = multi-gen, 4 = direct drive', deriv_ignore=True)
    crane = Bool(iotype='in', desc='flag for presence of crane', deriv_ignore=True)
    bevel = Int(0, iotype='in', desc='Flag for the presence of a bevel stage - 1 if present, 0 if not')
    gear_configuration = Str(iotype='in', desc='tring that represents the configuration of the gearbox (stage number and types)')

    # outputs
    nacelle_mass = Float(iotype='out', units='kg', desc='nacelle mass')
    nacelle_cm = Array(iotype='out', units='m', desc='center of mass of nacelle from tower top in yaw-aligned coordinate system')
    nacelle_I = Array(iotype='out', units='kg*m**2', desc='mass moments of inertia for nacelle [Ixx, Iyy, Izz, Ixy, Ixz, Iyz] about its center of mass')
    low_speed_shaft_mass = Float(iotype='out', units='kg', desc='component mass')
    main_bearing_mass = Float(iotype='out', units='kg', desc='component mass')
    second_bearing_mass = Float(iotype='out', units='kg', desc='component mass')
    gearbox_mass = Float(iotype='out', units='kg', desc='component mass')
    high_speed_side_mass = Float(iotype='out', units='kg', desc='component mass')
    generator_mass = Float(iotype='out', units='kg', desc='component mass')
    bedplate_mass = Float(iotype='out', units='kg', desc='component mass')
    yaw_system_mass = Float(iotype='out', units='kg', desc='component mass')

#-------------------------------------------------------------------------------------


class NacelleSE(NacelleBase):
    '''
       NacelleSE class
          The NacelleSE class is used to represent the nacelle system of a wind turbine.
    '''


    def configure(self):

        # select components
        self.add('above_yaw_massAdder', AboveYawMassAdder())
        self.add('nacelleSystem', NacelleSystemAdder())
        self.add('lowSpeedShaft', LowSpeedShaft())
        self.add('mainBearing', MainBearing())
        self.add('secondBearing', SecondBearing())
        self.add('gearbox', Gearbox())
        self.add('highSpeedSide', HighSpeedSide())
        self.add('generator', Generator())
        self.add('bedplate', Bedplate())
        self.add('yawSystem', YawSystem())

        # workflow
        self.driver.workflow.add(['above_yaw_massAdder', 'nacelleSystem', 'lowSpeedShaft', 'mainBearing', 'secondBearing', 'gearbox', 'highSpeedSide', 'generator', 'bedplate', 'yawSystem'])

        # connect inputs
        self.connect('rotor_diameter', ['lowSpeedShaft.rotor_diameter', 'mainBearing.rotor_diameter', 'secondBearing.rotor_diameter', 'gearbox.rotor_diameter', 'highSpeedSide.rotor_diameter',
                     'generator.rotor_diameter', 'bedplate.rotor_diameter', 'yawSystem.rotor_diameter'])
        self.connect('rotor_torque', ['lowSpeedShaft.rotor_torque', 'gearbox.rotor_torque', 'highSpeedSide.rotor_torque', 'bedplate.rotor_torque'])
        self.connect('rotor_mass', ['lowSpeedShaft.rotor_mass', 'bedplate.rotor_mass'])
        self.connect('rotor_speed', ['mainBearing.rotor_speed', 'secondBearing.rotor_speed'])
        self.connect('rotor_thrust', ['bedplate.rotor_thrust', 'yawSystem.rotor_thrust'])
        self.connect('tower_top_diameter', ['bedplate.tower_top_diameter', 'yawSystem.tower_top_diameter'])
        self.connect('machine_rating', ['generator.machine_rating', 'above_yaw_massAdder.machine_rating'])
        self.connect('drivetrain_design', ['gearbox.drivetrain_design', 'generator.drivetrain_design', 'bedplate.drivetrain_design'])
        self.connect('gear_ratio', ['gearbox.gear_ratio', 'generator.gear_ratio', 'highSpeedSide.gear_ratio'])
        self.connect('gear_configuration', 'gearbox.gear_configuration')
        self.connect('bevel', 'gearbox.bevel')
        self.connect('crane', 'above_yaw_massAdder.crane')


        # connect components
        self.connect('lowSpeedShaft.design_torque', ['mainBearing.lss_design_torque', 'secondBearing.lss_design_torque'])
        self.connect('lowSpeedShaft.diameter', ['mainBearing.lss_diameter', 'secondBearing.lss_diameter', 'highSpeedSide.lss_diameter'])
        self.connect('bedplate.length', 'above_yaw_massAdder.bedplate_length')
        self.connect('bedplate.width', 'above_yaw_massAdder.bedplate_width')

        self.connect('lowSpeedShaft.mass', ['mainBearing.lss_mass', 'secondBearing.lss_mass', 'above_yaw_massAdder.lss_mass', 'nacelleSystem.lss_mass'])
        self.connect('mainBearing.mass', ['above_yaw_massAdder.main_bearing_mass', 'nacelleSystem.main_bearing_mass'])
        self.connect('secondBearing.mass', ['above_yaw_massAdder.second_bearing_mass', 'nacelleSystem.second_bearing_mass'])
        self.connect('gearbox.mass', ['above_yaw_massAdder.gearbox_mass', 'nacelleSystem.gearbox_mass'])
        self.connect('highSpeedSide.mass', ['above_yaw_massAdder.hss_mass', 'nacelleSystem.hss_mass'])
        self.connect('generator.mass', ['above_yaw_massAdder.generator_mass', 'nacelleSystem.generator_mass'])
        self.connect('bedplate.mass', ['above_yaw_massAdder.bedplate_mass', 'nacelleSystem.bedplate_mass'])
        self.connect('above_yaw_massAdder.mainframe_mass', 'nacelleSystem.mainframe_mass')
        self.connect('yawSystem.mass', ['nacelleSystem.yawMass'])
        self.connect('above_yaw_massAdder.above_yaw_mass', ['yawSystem.above_yaw_mass', 'nacelleSystem.above_yaw_mass'])

        self.connect('lowSpeedShaft.cm', ['nacelleSystem.lss_cm'])
        self.connect('mainBearing.cm', 'nacelleSystem.main_bearing_cm')
        self.connect('secondBearing.cm', 'nacelleSystem.second_bearing_cm')
        self.connect('gearbox.cm', ['nacelleSystem.gearbox_cm'])
        self.connect('highSpeedSide.cm', ['nacelleSystem.hss_cm'])
        self.connect('generator.cm', ['nacelleSystem.generator_cm'])
        self.connect('bedplate.cm', ['nacelleSystem.bedplate_cm'])

        self.connect('lowSpeedShaft.I', ['nacelleSystem.lss_I'])
        self.connect('mainBearing.I', 'nacelleSystem.main_bearing_I')
        self.connect('secondBearing.I', 'nacelleSystem.second_bearing_I')
        self.connect('gearbox.I', ['nacelleSystem.gearbox_I'])
        self.connect('highSpeedSide.I', ['nacelleSystem.hss_I'])
        self.connect('generator.I', ['nacelleSystem.generator_I'])
        self.connect('bedplate.I', ['nacelleSystem.bedplate_I'])

        # connect outputs
        self.connect('nacelleSystem.nacelle_mass', 'nacelle_mass')
        self.connect('nacelleSystem.nacelle_cm', 'nacelle_cm')
        self.connect('nacelleSystem.nacelle_I', 'nacelle_I')
        self.connect('lowSpeedShaft.mass', 'low_speed_shaft_mass')
        self.connect('mainBearing.mass', 'main_bearing_mass')
        self.connect('secondBearing.mass', 'second_bearing_mass')
        self.connect('gearbox.mass', 'gearbox_mass')
        self.connect('highSpeedSide.mass', 'high_speed_side_mass')
        self.connect('generator.mass', 'generator_mass')
        self.connect('bedplate.mass', 'bedplate_mass')
        self.connect('yawSystem.mass', 'yaw_system_mass')

    '''def getNacelleComponentMasses(self):
        """ Returns detailed nacelle assembly masses

        detailedMasses : array_like of float
           detailed masses for nacelle components
        """

        detailedMasses = [self.lss_mass, self.mainBearingsMass, self.gearbox_mass, self.hss_mass, self.generator_mass, self.vs_electronics_mass, \
                self.electrical_mass, self.hvac_mass, \
                self.ControlsMass, self.yawMass, self.mainframe_mass, self.cover_mass]

        return detailedMasses'''

#------------------------------------------------------------------

class NacelleSE_drive(NacelleBase):

    '''
       NacelleSE class
          The NacelleSE class is used to represent the nacelle system of a wind turbine.
    '''

    # parameters
    rotor_bending_moment = Float(iotype='in', units='N*m', desc='rotor aerodynamic bending moment')
    shaft_angle = Float(iotype='in', units='deg', desc='Angle of the LSS inclindation with respect to the horizontal')
    shaft_length = Float(iotype='in', units='m', desc='length of low speed shaft')
    shaftD1 = Float(iotype='in', desc='Fraction of LSS distance from gearbox to downwind main bearing')
    shaftD2 = Float(iotype='in', desc='raction of LSS distance from gearbox to upwind main bearing')
    shaft_ratio = Float(iotype='in', desc='Ratio of inner diameter to outer diameter.  Leave zero for solid LSS')
    rotorRatedRPM = Float(iotype='in', units='rpm', desc='Speed of rotor at rated power')
    #gbxPower = Float(iotype='in', units='kW', desc='gearbox rated power')
    #eff = Float(iotype='in', desc='Fraction of LSS distance from gearbox to downwind main bearing')
    Np = Array(np.array([0.0,0.0,0.0,]), iotype='in', desc='number of planets in each stage')
    ratio_type=Str(iotype='in', desc='optimal or empirical stage ratios')
    shaft_type = Str(iotype='in', desc = 'normal or short shaft length')
    uptower_transformer = Bool(iotype = 'in', desc = 'Boolean stating if transformer is uptower')



    # potential additional new parameters

    '''        name : str
          Name of the gearbox.
        gbxPower : float
          Rated power of the gearbox [W].
        ratio : float
          Overall gearbox speedup ratio.
        gearConfig : str
          String describing configuration of each gear stage.  Use 'e' for epicyclic and 'p' for parallel, for example 'eep' would be epicyclic-epicyclic-parallel.
        Np : array
          Array describing the number of planets in each stage.  For example if gearConfig is 'eep' Np could be [3 3 1].
        eff : float
          Mechanical efficiency of the gearbox.
        ratio_type : str
          Describes how individual stage ratios will be calculated.  Can be 'empirical' which uses the Sunderland model, or 'optimal' which finds the stage ratios that minimize overall mass.
        shaft_type : str
          Describes the shaft type and applies a corresponding application factor.  Can be 'normal' or 'short'.
        uptower_transformer : int
          Determines if the transformer is uptower ('1') or downtower ('0').
        yaw_motors_number : int
          Number of yaw motors.'''

    def configure(self):

        # select components
        self.add('above_yaw_massAdder', AboveYawMassAdder())
        self.add('nacelleSystem', NacelleSystemAdder())
        self.add('lowSpeedShaft', LowSpeedShaft_drive())
        self.add('mainBearing', MainBearing())
        self.add('secondBearing',SecondBearing())
        self.add('gearbox', Gearbox())
        self.add('highSpeedSide', HighSpeedSide())
        self.add('generator', Generator())
        self.add('bedplate', Bedplate_drive())
        self.add('yawSystem', YawSystem_drive())

        # workflow
        self.driver.workflow.add(['above_yaw_massAdder', 'nacelleSystem', 'lowSpeedShaft', 'mainBearing', 'secondBearing', 'gearbox', 'highSpeedSide', 'generator', 'bedplate', 'yawSystem'])

        # connect inputs
        self.connect('rotor_diameter', ['lowSpeedShaft.rotor_diameter', 'mainBearing.rotor_diameter', 'secondBearing.rotor_diameter', 'gearbox.rotor_diameter', 'highSpeedSide.rotor_diameter', \
                     'generator.rotor_diameter', 'bedplate.rotor_diameter', 'yawSystem.rotor_diameter'])
        self.connect('rotor_bending_moment', 'lowSpeedShaft.rotor_bending_moment')

        self.connect('rotor_torque', ['lowSpeedShaft.rotor_torque', 'gearbox.rotor_torque', 'highSpeedSide.rotor_torque'])

        self.connect('rotor_mass', 'lowSpeedShaft.rotor_mass')
        self.connect('rotor_speed', ['mainBearing.rotor_speed', 'secondBearing.rotor_speed'])

        self.connect('rotor_thrust', 'yawSystem.rotor_thrust')
        self.connect('tower_top_diameter', ['bedplate.tower_top_diameter', 'yawSystem.tower_top_diameter'])
        self.connect('machine_rating', ['generator.machine_rating', 'above_yaw_massAdder.machine_rating', 'lowSpeedShaft.machine_rating'])

        self.connect('drivetrain_design', 'generator.drivetrain_design')
        self.connect('gear_ratio', ['gearbox.gear_ratio', 'generator.gear_ratio', 'highSpeedSide.gear_ratio'])
        self.connect('gear_configuration', 'gearbox.gear_configuration')

        self.connect('crane', 'above_yaw_massAdder.crane')
        self.connect('shaft_angle', 'lowSpeedShaft.shaft_angle')
        self.connect('shaft_length', ['lowSpeedShaft.shaft_length', 'bedplate.shaft_length'])
        self.connect('shaftD1', 'lowSpeedShaft.shaftD1')
        self.connect('shaftD2', 'lowSpeedShaft.shaftD2')
        self.connect('shaft_ratio', 'lowSpeedShaft.shaft_ratio')
        self.connect('rotorRatedRPM', 'lowSpeedShaft.rotor_speed')


        # connect components
        self.connect('lowSpeedShaft.design_torque', ['mainBearing.lss_design_torque', 'secondBearing.lss_design_torque'])
        self.connect('lowSpeedShaft.diameter', ['mainBearing.lss_diameter', 'secondBearing.lss_diameter', 'highSpeedSide.lss_diameter'])
        self.connect('bedplate.length', 'above_yaw_massAdder.bedplate_length')
        self.connect('bedplate.width', 'above_yaw_massAdder.bedplate_width')

        self.connect('lowSpeedShaft.mass', ['mainBearing.lss_mass', 'secondBearing.lss_mass', 'above_yaw_massAdder.lss_mass', 'nacelleSystem.lss_mass'])
        self.connect('mainBearing.mass', ['above_yaw_massAdder.main_bearing_mass', 'nacelleSystem.main_bearing_mass'])
        self.connect('secondBearing.mass', ['above_yaw_massAdder.second_bearing_mass', 'nacelleSystem.second_bearing_mass'])
        self.connect('gearbox.mass', ['above_yaw_massAdder.gearbox_mass', 'nacelleSystem.gearbox_mass'])
        self.connect('highSpeedSide.mass', ['above_yaw_massAdder.hss_mass', 'nacelleSystem.hss_mass'])
        self.connect('generator.mass', ['above_yaw_massAdder.generator_mass', 'nacelleSystem.generator_mass'])
        self.connect('bedplate.mass', ['above_yaw_massAdder.bedplate_mass', 'nacelleSystem.bedplate_mass'])
        self.connect('above_yaw_massAdder.mainframe_mass', 'nacelleSystem.mainframe_mass')
        self.connect('yawSystem.mass', ['nacelleSystem.yawMass'])
        self.connect('above_yaw_massAdder.above_yaw_mass', ['yawSystem.above_yaw_mass', 'nacelleSystem.above_yaw_mass'])

        self.connect('lowSpeedShaft.cm', ['nacelleSystem.lss_cm'])
        self.connect('mainBearing.cm', 'nacelleSystem.main_bearing_cm')
        self.connect('secondBearing.cm', 'nacelleSystem.second_bearing_cm')
        self.connect('gearbox.cm', ['nacelleSystem.gearbox_cm'])
        self.connect('highSpeedSide.cm', ['nacelleSystem.hss_cm'])
        self.connect('generator.cm', ['nacelleSystem.generator_cm'])
        self.connect('bedplate.cm', ['nacelleSystem.bedplate_cm'])

        self.connect('lowSpeedShaft.I', ['nacelleSystem.lss_I'])
        self.connect('mainBearing.I', 'nacelleSystem.main_bearing_I')
        self.connect('secondBearing.I', 'nacelleSystem.second_bearing_I')
        self.connect('gearbox.I', ['nacelleSystem.gearbox_I'])
        self.connect('highSpeedSide.I', ['nacelleSystem.hss_I'])
        self.connect('generator.I', ['nacelleSystem.generator_I'])
        self.connect('bedplate.I', ['nacelleSystem.bedplate_I'])

        # create passthroughs
        self.connect('nacelleSystem.nacelle_mass', 'nacelle_mass')
        self.connect('nacelleSystem.nacelle_cm', 'nacelle_cm')
        self.connect('nacelleSystem.nacelle_I', 'nacelle_I')


    '''def getNacelleComponentMasses(self):
        """ Returns detailed nacelle assembly masses

        detailedMasses : array_like of float
           detailed masses for nacelle components
        """

        detailedMasses = [self.lss_mass, self.mainBearingsMass, self.gearbox_mass, self.hss_mass, self.generator_mass, self.vs_electronics_mass, \
                self.electrical_mass, self.hvac_mass, \
                self.ControlsMass, self.yawMass, self.mainframe_mass, self.cover_mass]

        return detailedMasses'''

#------------------------------------------------------------------

class NacelleSE_drive3pt(NacelleBase):

    '''
       NacelleSE class
          The NacelleSE class is used to represent the nacelle system of a wind turbine.
    '''

    # parameters
    rotor_bending_moment_x = Float(iotype='in', units='N*m', desc='The bending moment about the x axis')
    rotor_bending_moment_y = Float(iotype='in', units='N*m', desc='The bending moment about the y axis')
    rotor_bending_moment_z = Float(iotype='in', units='N*m', desc='The bending moment about the z axis')
    rotor_force_x = Float(iotype='in', units='N', desc='The force along the x axis applied at hub center')
    rotor_force_y = Float(iotype='in', units='N', desc='The force along the y axis applied at hub center')
    rotor_force_z = Float(iotype='in', units='N', desc='The force along the z axis applied at hub center')
    shaft_angle = Float(iotype='in', units='deg', desc='Angle of the LSS inclindation with respect to the horizontal')
    shaft_ratio = Float(iotype='in', desc='Ratio of inner diameter to outer diameter.  Leave zero for solid LSS')
    rotorRatedRPM = Float(iotype='in', units='rpm', desc='Speed of rotor at rated power')
    Np = Array(np.array([0.0,0.0,0.0,]), iotype='in', desc='number of planets in each stage')
    ratio_type=Str(iotype='in', desc='optimal or empirical stage ratios')
    shaft_type = Str(iotype='in', desc = 'normal or short shaft length')
    uptower_transformer = Bool(iotype = 'in', desc = 'Boolean stating if transformer is uptower')
    shrink_disc_mass = Float(iotype='in', units='kg', desc='Mass of the shrink disc')


    # potential additional new parameters

    '''        name : str
          Name of the gearbox.
        gbxPower : float
          Rated power of the gearbox [W].
        ratio : float
          Overall gearbox speedup ratio.
        gearConfig : str
          String describing configuration of each gear stage.  Use 'e' for epicyclic and 'p' for parallel, for example 'eep' would be epicyclic-epicyclic-parallel.
        Np : array
          Array describing the number of planets in each stage.  For example if gearConfig is 'eep' Np could be [3 3 1].
        eff : float
          Mechanical efficiency of the gearbox.
        ratio_type : str
          Describes how individual stage ratios will be calculated.  Can be 'empirical' which uses the Sunderland model, or 'optimal' which finds the stage ratios that minimize overall mass.
        shaft_type : str
          Describes the shaft type and applies a corresponding application factor.  Can be 'normal' or 'short'.
        uptower_transformer : int
          Determines if the transformer is uptower ('1') or downtower ('0').
        yaw_motors_number : int
          Number of yaw motors.'''

    def configure(self):

        # select components
        self.add('above_yaw_massAdder', AboveYawMassAdder())
        self.add('nacelleSystem', NacelleSystemAdder())
        self.add('lowSpeedShaft', LowSpeedShaft_drive3pt())
        self.add('mainBearing', MainBearing())
        self.add('secondBearing',SecondBearing())
        self.add('gearbox', Gearbox())
        self.add('highSpeedSide', HighSpeedSide())
        self.add('generator', Generator())
        self.add('bedplate', Bedplate_drive())
        self.add('yawSystem', YawSystem_drive())

        # workflow
        self.driver.workflow.add(['above_yaw_massAdder', 'nacelleSystem', 'lowSpeedShaft', 'mainBearing', 'secondBearing', 'gearbox', 'highSpeedSide', 'generator', 'bedplate', 'yawSystem'])

        # connect inputs
        self.connect('rotor_diameter', ['lowSpeedShaft.rotor_diameter', 'mainBearing.rotor_diameter', 'secondBearing.rotor_diameter', 'gearbox.rotor_diameter', 'highSpeedSide.rotor_diameter', \
                     'generator.rotor_diameter', 'bedplate.rotor_diameter', 'yawSystem.rotor_diameter'])
        self.connect('rotor_bending_moment_x', 'lowSpeedShaft.rotor_bending_moment_x')
        self.connect('rotor_bending_moment_y', 'lowSpeedShaft.rotor_bending_moment_y')
        self.connect('rotor_bending_moment_z', 'lowSpeedShaft.rotor_bending_moment_z')
        self.connect('rotor_force_x', 'lowSpeedShaft.rotor_force_x')
        self.connect('rotor_force_y', 'lowSpeedShaft.rotor_force_y')
        self.connect('rotor_force_z', 'lowSpeedShaft.rotor_force_z')
        self.connect('rotor_torque', ['gearbox.rotor_torque', 'highSpeedSide.rotor_torque'])
        self.connect('rotor_mass', 'lowSpeedShaft.rotor_mass')
        self.connect('rotor_speed', ['mainBearing.rotor_speed', 'secondBearing.rotor_speed'])
        self.connect('rotor_thrust', 'yawSystem.rotor_thrust')
        self.connect('tower_top_diameter', ['bedplate.tower_top_diameter', 'yawSystem.tower_top_diameter'])
        self.connect('machine_rating', ['generator.machine_rating', 'above_yaw_massAdder.machine_rating', 'lowSpeedShaft.machine_rating'])
        self.connect('drivetrain_design', 'generator.drivetrain_design')
        self.connect('gear_ratio', ['gearbox.gear_ratio', 'generator.gear_ratio', 'highSpeedSide.gear_ratio'])
        self.connect('gear_configuration', 'gearbox.gear_configuration')
        self.connect('crane', 'above_yaw_massAdder.crane')
        self.connect('shaft_angle', 'lowSpeedShaft.shaft_angle')
        self.connect('shaft_ratio', 'lowSpeedShaft.shaft_ratio')
        self.connect('shrink_disc_mass', 'lowSpeedShaft.shrink_disc_mass')
        self.connect('carrier_mass', 'lowSpeedShaft.carrier_mass')

        # connect components
        self.connect('lowSpeedShaft.design_torque', ['mainBearing.lss_design_torque', 'secondBearing.lss_design_torque'])
        self.connect('lowSpeedShaft.diameter', ['mainBearing.lss_diameter', 'secondBearing.lss_diameter', 'highSpeedSide.lss_diameter'])
        self.connect('lowSpeedShaft.length', 'bedplate.shaft_length')
        self.connect('bedplate.length', 'above_yaw_massAdder.bedplate_length')
        self.connect('bedplate.width', 'above_yaw_massAdder.bedplate_width')

        self.connect('lowSpeedShaft.mass', ['mainBearing.lss_mass', 'secondBearing.lss_mass', 'above_yaw_massAdder.lss_mass', 'nacelleSystem.lss_mass'])
        self.connect('mainBearing.mass', ['above_yaw_massAdder.main_bearing_mass', 'nacelleSystem.main_bearing_mass'])
        self.connect('secondBearing.mass', ['above_yaw_massAdder.second_bearing_mass', 'nacelleSystem.second_bearing_mass'])
        self.connect('gearbox.mass', ['lowSpeedShaft.gearbox_mass', 'above_yaw_massAdder.gearbox_mass', 'nacelleSystem.gearbox_mass'])
        self.connect('highSpeedSide.mass', ['above_yaw_massAdder.hss_mass', 'nacelleSystem.hss_mass'])
        self.connect('generator.mass', ['above_yaw_massAdder.generator_mass', 'nacelleSystem.generator_mass'])
        self.connect('bedplate.mass', ['above_yaw_massAdder.bedplate_mass', 'nacelleSystem.bedplate_mass'])
        self.connect('above_yaw_massAdder.mainframe_mass', 'nacelleSystem.mainframe_mass')
        self.connect('yawSystem.mass', ['nacelleSystem.yawMass'])
        self.connect('above_yaw_massAdder.above_yaw_mass', ['yawSystem.above_yaw_mass', 'nacelleSystem.above_yaw_mass'])

        self.connect('lowSpeedShaft.cm', ['nacelleSystem.lss_cm'])
        self.connect('mainBearing.cm', 'nacelleSystem.main_bearing_cm')
        self.connect('secondBearing.cm', 'nacelleSystem.second_bearing_cm')
        self.connect('gearbox.cm', ['nacelleSystem.gearbox_cm'])
        self.connect('highSpeedSide.cm', ['nacelleSystem.hss_cm'])
        self.connect('generator.cm', ['nacelleSystem.generator_cm'])
        self.connect('bedplate.cm', ['nacelleSystem.bedplate_cm'])

        self.connect('lowSpeedShaft.I', ['nacelleSystem.lss_I'])
        self.connect('mainBearing.I', 'nacelleSystem.main_bearing_I')
        self.connect('secondBearing.I', 'nacelleSystem.second_bearing_I')
        self.connect('gearbox.I', ['nacelleSystem.gearbox_I'])
        self.connect('highSpeedSide.I', ['nacelleSystem.hss_I'])
        self.connect('generator.I', ['nacelleSystem.generator_I'])
        self.connect('bedplate.I', ['nacelleSystem.bedplate_I'])

        # create passthroughs
        self.connect('nacelleSystem.nacelle_mass', 'nacelle_mass')
        self.connect('nacelleSystem.nacelle_cm', 'nacelle_cm')
        self.connect('nacelleSystem.nacelle_I', 'nacelle_I')


    '''def getNacelleComponentMasses(self):
        """ Returns detailed nacelle assembly masses

        detailedMasses : array_like of float
           detailed masses for nacelle components
        """

        detailedMasses = [self.lss_mass, self.mainBearingsMass, self.gearbox_mass, self.hss_mass, self.generator_mass, self.vs_electronics_mass, \
                self.electrical_mass, self.hvac_mass, \
                self.ControlsMass, self.yawMass, self.mainframe_mass, self.cover_mass]

        return detailedMasses'''

#------------------------------------------------------------------

class NacelleSE_drive4pt(NacelleBase):

    '''
       NacelleSE class
          The NacelleSE class is used to represent the nacelle system of a wind turbine.
    '''

    # variables
    rotor_bending_moment_x = Float(iotype='in', units='N*m', desc='The bending moment about the x axis')
    rotor_bending_moment_y = Float(iotype='in', units='N*m', desc='The bending moment about the y axis')
    rotor_bending_moment_z = Float(iotype='in', units='N*m', desc='The bending moment about the z axis')
    rotor_force_x = Float(iotype='in', units='N', desc='The force along the x axis applied at hub center')
    rotor_force_y = Float(iotype='in', units='N', desc='The force along the y axis applied at hub center')
    rotor_force_z = Float(iotype='in', units='N', desc='The force along the z axis applied at hub center')
    shaft_angle = Float(iotype='in', units='deg', desc='Angle of the LSS inclindation with respect to the horizontal') # Bedplate tilting angle
    shaft_ratio = Float(iotype='in', desc='Ratio of inner diameter to outer diameter.  Leave zero for solid LSS')
    rotorRatedRPM = Float(iotype='in', units='rpm', desc='Speed of rotor at rated power')

    # parameters
    Np = Array(np.array([0.0,0.0,0.0,]), iotype='in', desc='number of planets in each stage')
    ratio_type=Str(iotype='in', desc='optimal or empirical stage ratios')
    shaft_type = Str(iotype='in', desc = 'normal or short shaft length')
    uptower_transformer = Bool(iotype = 'in', desc = 'Boolean stating if transformer is uptower')
    shrink_disc_mass = Float(iotype='in', units='kg', desc='Mass of the shrink disc')
    carrier_mass = Float(iotype='in', units='kg', desc='Carrier mass')
    mb1Type = Str(iotype='in',desc='Main bearing type: CARB, TRB or SRB')
    mb2Type = Str(iotype='in',desc='Second bearing type: CARB, TRB or SRB')


    '''        name : str
          Name of the gearbox.
        gbxPower : float
          Rated power of the gearbox [W].
        ratio : float
          Overall gearbox speedup ratio.
        gearConfig : str
          String describing configuration of each gear stage.  Use 'e' for epicyclic and 'p' for parallel, for example 'eep' would be epicyclic-epicyclic-parallel.
        Np : array
          Array describing the number of planets in each stage.  For example if gearConfig is 'eep' Np could be [3 3 1].
        eff : float
          Mechanical efficiency of the gearbox.
        ratio_type : str
          Describes how individual stage ratios will be calculated.  Can be 'empirical' which uses the Sunderland model, or 'optimal' which finds the stage ratios that minimize overall mass.
        shaft_type : str
          Describes the shaft type and applies a corresponding application factor.  Can be 'normal' or 'short'.
        uptower_transformer : int
          Determines if the transformer is uptower ('1') or downtower ('0').
        yaw_motors_number : int
          Number of yaw motors.'''

    def configure(self):

        # select components
        self.add('above_yaw_massAdder', AboveYawMassAdder())
        self.add('nacelleSystem', NacelleSystemAdder())
        self.add('lowSpeedShaft', LowSpeedShaft_drive4pt())
        self.add('mainBearing', MainBearing_drive())
        self.add('secondBearing',SecondBearing_drive())
        self.add('gearbox', Gearbox_drive())
        self.add('highSpeedSide', HighSpeedSide())
        self.add('generator', Generator())
        self.add('bedplate', Bedplate_drive())
        self.add('yawSystem', YawSystem_drive())

        # workflow
        self.driver.workflow.add(['above_yaw_massAdder', 'nacelleSystem', 'lowSpeedShaft', 'mainBearing', 'secondBearing', 'gearbox', 'highSpeedSide', 'generator', 'bedplate', 'yawSystem'])

        # connect inputs
        self.connect('rotor_diameter', ['lowSpeedShaft.rotor_diameter', 'mainBearing.rotor_diameter', 'secondBearing.rotor_diameter', 'gearbox.rotor_diameter', 'highSpeedSide.rotor_diameter', \
                     'generator.rotor_diameter', 'bedplate.rotor_diameter', 'yawSystem.rotor_diameter'])
        self.connect('rotor_bending_moment_x', ['lowSpeedShaft.rotor_bending_moment_x'])
        self.connect('rotor_bending_moment_y', ['bedplate.rotor_bending_moment_y','lowSpeedShaft.rotor_bending_moment_y'])
        self.connect('rotor_bending_moment_z', 'lowSpeedShaft.rotor_bending_moment_z')
        self.connect('rotor_force_x', 'lowSpeedShaft.rotor_force_x')
        self.connect('rotor_force_y', 'lowSpeedShaft.rotor_force_y')
        self.connect('rotor_force_z', ['bedplate.rotor_force_z','lowSpeedShaft.rotor_force_z'])
        self.connect('rotor_torque', ['gearbox.rotor_torque', 'highSpeedSide.rotor_torque']) # Need to address internal torque calculations...
        self.connect('rotor_mass', ['bedplate.rotor_mass','lowSpeedShaft.rotor_mass'])
        self.connect('rotor_thrust', 'yawSystem.rotor_thrust')
        self.connect('tower_top_diameter', ['bedplate.tower_top_diameter', 'yawSystem.tower_top_diameter'])
        self.connect('machine_rating', ['bedplate.machine_rating', 'generator.machine_rating', 'above_yaw_massAdder.machine_rating', 'lowSpeedShaft.machine_rating'])
        self.connect('drivetrain_design', 'generator.drivetrain_design')
        self.connect('gear_ratio', ['gearbox.gear_ratio', 'generator.gear_ratio', 'highSpeedSide.gear_ratio'])
        self.connect('gear_configuration', 'gearbox.gear_configuration')
        self.connect('crane', 'above_yaw_massAdder.crane')
        self.connect('shaft_angle', 'lowSpeedShaft.shaft_angle')
        self.connect('shaft_ratio', 'lowSpeedShaft.shaft_ratio')
        self.connect('shrink_disc_mass', 'lowSpeedShaft.shrink_disc_mass')
        self.connect('carrier_mass', 'lowSpeedShaft.carrier_mass')
        self.connect('mb1Type', ['mainBearing.bearing_type', 'lowSpeedShaft.mb1Type'])
        self.connect('mb2Type', ['secondBearing.bearing_type', 'lowSpeedShaft.mb2Type'])
        self.connect('Np', 'gearbox.Np')
        self.connect('ratio_type', 'gearbox.ratio_type')
        self.connect('shaft_type', 'gearbox.shaft_type')

        # connect components
        self.connect('lowSpeedShaft.design_torque', ['mainBearing.lss_design_torque', 'secondBearing.lss_design_torque'])
        self.connect('lowSpeedShaft.diameter1', ['mainBearing.lss_diameter', 'highSpeedSide.lss_diameter'])
        self.connect('lowSpeedShaft.diameter2', 'secondBearing.lss_diameter')
        self.connect('lowSpeedShaft.length', 'bedplate.shaft_length')
        self.connect('bedplate.length', 'above_yaw_massAdder.bedplate_length')
        self.connect('bedplate.width', 'above_yaw_massAdder.bedplate_width')

        self.connect('lowSpeedShaft.mass', ['bedplate.lss_mass','above_yaw_massAdder.lss_mass', 'nacelleSystem.lss_mass'])
        self.connect('mainBearing.mass', ['bedplate.mb1_mass','above_yaw_massAdder.main_bearing_mass', 'nacelleSystem.main_bearing_mass'])
        self.connect('secondBearing.mass', ['bedplate.mb2_mass','above_yaw_massAdder.second_bearing_mass', 'nacelleSystem.second_bearing_mass'])
        self.connect('gearbox.mass', ['lowSpeedShaft.gearbox_mass', 'above_yaw_massAdder.gearbox_mass', 'nacelleSystem.gearbox_mass'])
        self.connect('highSpeedSide.mass', ['bedplate.hss_mass','above_yaw_massAdder.hss_mass', 'nacelleSystem.hss_mass'])
        self.connect('generator.mass', ['bedplate.generator_mass','above_yaw_massAdder.generator_mass', 'nacelleSystem.generator_mass'])
        self.connect('bedplate.mass', ['above_yaw_massAdder.bedplate_mass', 'nacelleSystem.bedplate_mass'])
        self.connect('above_yaw_massAdder.mainframe_mass', 'nacelleSystem.mainframe_mass')
        self.connect('yawSystem.mass', ['nacelleSystem.yawMass'])
        self.connect('above_yaw_massAdder.above_yaw_mass', ['yawSystem.above_yaw_mass', 'nacelleSystem.above_yaw_mass'])

        self.connect('lowSpeedShaft.cm', ['nacelleSystem.lss_cm'])
        self.connect('lowSpeedShaft.cm[0]', 'bedplate.lss_location')
        self.connect('mainBearing.cm', 'nacelleSystem.main_bearing_cm')
        self.connect('mainBearing.cm[0]', 'bedplate.mb1_location')
        self.connect('secondBearing.cm', 'nacelleSystem.second_bearing_cm')
        self.connect('secondBearing.cm[0]', 'bedplate.mb2_location')
        self.connect('gearbox.cm', ['nacelleSystem.gearbox_cm'])
        self.connect('highSpeedSide.cm', ['nacelleSystem.hss_cm'])
        self.connect('highSpeedSide.cm[0]','bedplate.hss_location')
        self.connect('generator.cm', ['nacelleSystem.generator_cm'])
        self.connect('generator.cm[0]','bedplate.generator_location')
        self.connect('bedplate.cm', ['nacelleSystem.bedplate_cm'])

        self.connect('lowSpeedShaft.I', ['nacelleSystem.lss_I'])
        self.connect('mainBearing.I', 'nacelleSystem.main_bearing_I')
        self.connect('secondBearing.I', 'nacelleSystem.second_bearing_I')
        self.connect('gearbox.I', ['nacelleSystem.gearbox_I'])
        self.connect('highSpeedSide.I', ['nacelleSystem.hss_I'])
        self.connect('generator.I', ['nacelleSystem.generator_I'])
        self.connect('bedplate.I', ['nacelleSystem.bedplate_I'])

        # create passthroughs
        self.connect('nacelleSystem.nacelle_mass', 'nacelle_mass')
        self.connect('nacelleSystem.nacelle_cm', 'nacelle_cm')
        self.connect('nacelleSystem.nacelle_I', 'nacelle_I')


    '''def getNacelleComponentMasses(self):
        """ Returns detailed nacelle assembly masses

        detailedMasses : array_like of float
           detailed masses for nacelle components
        """

        detailedMasses = [self.lss_mass, self.mainBearingsMass, self.gearbox_mass, self.hss_mass, self.generator_mass, self.vs_electronics_mass, \
                self.electrical_mass, self.hvac_mass, \
                self.ControlsMass, self.yawMass, self.mainframe_mass, self.cover_mass]

        return detailedMasses'''

#------------------------------------------------------------------

def example():

    # test of module for turbine data set

    # NREL 5 MW Rotor Variables
    print '----- NREL 5 MW Turbine -----'
    nace = NacelleSE_drive4pt()
    nace.rotor_diameter = 126.0 # m
    nace.rotor_speed = 12.1 # m/s
    nace.machine_rating = 5000.0
    DrivetrainEfficiency = 0.95
    nace.rotor_torque =  1.5 * (nace.machine_rating * 1000 / DrivetrainEfficiency) / (nace.rotor_speed * (pi / 30)) # 6.35e6 #4365248.74 # Nm
    nace.rotor_thrust = 599610.0 #500930.84 # N
    nace.rotor_mass = 0.0 #142585.75 # kg
    nace.rotorRatedRPM = 12.1 #rpm
    nace.rotor_bending_moment = -16665000.0 #DLC 1.4
    nace.rotor_bending_moment_x = 3.3e5 #4365248.74
    nace.rotor_bending_moment_y = -16665000.0 #14700000.0
    nace.rotor_bending_moment_z = 2896300.0 #0.0
    nace.rotor_force_x = 599610.0 #500930.84
    nace.rotor_force_y = 186780.0 #0.0
    nace.rotor_force_z = -842710.0 #1e6
    # nace.rotor_bending_moment_x = 4.3268e6
    # nace.rotor_bending_moment_y = -1.0552e6
    # nace.rotor_bending_moment_z = -1.4607e7
    # nace.rotor_force_x = 3.0051e5
    # nace.rotor_force_y = 4.3227e4
    # nace.rotor_force_z = -1.2381e6

    # NREL 5 MW Drivetrain variables
    nace.drivetrain_design = 1 # geared 3-stage Gearbox with induction generator machine
    nace.machine_rating = 5000.0 # kW
    nace.gear_ratio = 96.76 # 97:1 as listed in the 5 MW reference document
    nace.gear_configuration = 'eep' # epicyclic-epicyclic-parallel
    #nace.bevel = 0 # no bevel stage
    nace.crane = True # onboard crane present
    nace.shaft_angle = 5.0 #deg
    nace.shaft_length = 3.383 #m
    nace.shaftD1 = 0.25
    nace.shaftD2 = 0.75
    nace.shaft_ratio = 0.10
    nace.Np = [3,3,1]
    nace.ratio_type = 'optimal'
    nace.shaft_type = 'normal'
    nace.uptower_transformer=True
    nace.shrink_disc_mass = 1000.0 # estimated
    nace.carrier_mass = 8000.0 # estimated
    nace.mb1Type = 'CARB'
    nace.mb2Type = 'SRB'


    # NREL 5 MW Tower Variables
    nace.tower_top_diameter = 3.78 # m

    nace.run()

    print 'Nacelle system model results'
    print 'Low speed shaft %8.1f kg %6.2f m Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
          % (nace.lowSpeedShaft.mass , nace.lowSpeedShaft.I[0], nace.lowSpeedShaft.I[1], nace.lowSpeedShaft.I[2], nace.lowSpeedShaft.cm[0], nace.lowSpeedShaft.cm[1], nace.lowSpeedShaft.cm[2])
    # 31257.3 kg
    print 'Main bearings   %8.1f kg ' % (nace.mainBearing.mass + nace.secondBearing.mass)
    # 9731.4 kg
    print 'Gearbox         %8.1f kg %6.2f Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
          % (nace.gearbox.mass, nace.gearbox.I[0], nace.gearbox.I[1], nace.gearbox.I[2], nace.gearbox.cm[0], nace.gearbox.cm[1], nace.gearbox.cm[2] )
    # 30237.6 kg
    print '     gearbox stage masses: %8.1f kg  %8.1f kg %8.1f kg' % (nace.gearbox.stage_masses[0], nace.gearbox.stage_masses[1], nace.gearbox.stage_masses[2])
    print 'High speed shaft & brakes  %8.1f kg %6.2f Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
          % (nace.highSpeedSide.mass, nace.highSpeedSide.I[0], nace.highSpeedSide.I[1], nace.highSpeedSide.I[2], nace.highSpeedSide.cm[0], nace.highSpeedSide.cm[1], nace.highSpeedSide.cm[2])
    # 1492.4 kg
    print 'Generator       %8.1f kg %6.2f Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
          % (nace.generator.mass, nace.generator.I[0], nace.generator.I[1], nace.generator.I[2], nace.generator.cm[0], nace.generator.cm[1], nace.generator.cm[2])
    # 16699.9 kg
    print 'Variable speed electronics %8.1f kg' % (nace.above_yaw_massAdder.vs_electronics_mass)
    # 0.0 kg
    print 'Overall mainframe %8.1f kg' % (nace.above_yaw_massAdder.mainframe_mass)
    # 96932.9 kg
    print '     Bedplate     %8.1f kg %6.2f Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
         % (nace.bedplate.mass, nace.bedplate.I[0], nace.bedplate.I[1], nace.bedplate.I[2], nace.bedplate.cm[0], nace.bedplate.cm[1], nace.bedplate.cm[2])
    print 'electrical connections  %8.1f kg' % (nace.above_yaw_massAdder.electrical_mass)
    # 0.0 kg
    print 'HVAC system     %8.1f kg' % (nace.above_yaw_massAdder.hvac_mass )
    # 400.0 kg
    print 'Nacelle cover:   %8.1f kg %6.2f m Height %6.2f m Width %6.2f m Length' % (nace.above_yaw_massAdder.cover_mass , nace.above_yaw_massAdder.height, nace.above_yaw_massAdder.width, nace.above_yaw_massAdder.length)
    # 9097.4 kg
    print 'Yaw system      %8.1f kg' % (nace.yawSystem.mass )
    # 11878.2 kg
    print 'Overall nacelle:  %8.1f kg cm %6.2f %6.2f %6.2f I %6.2f %6.2f %6.2f' % (nace.nacelle_mass, nace.nacelle_cm[0], nace.nacelle_cm[1], nace.nacelle_cm[2], nace.nacelle_I[0], nace.nacelle_I[1], nace.nacelle_I[2]  )
    # 207727.1


def example2():

    # WindPACT 1.5 MW Drivetrain variables
    nace = NacelleSE()
    nace.drivetrain_design = 1 # geared 3-stage Gearbox with induction generator machine
    nace.machine_rating = 1500 # machine rating [kW]
    nace.gear_ratio = 87.965
    nace.gear_configuration = 'epp'
    nace.bevel = 0
    nace.crane = True

    # WindPACT 1.5 MW Rotor Variables
    airdensity = 1.225 # air density [kg / m^3]
    MaxTipSpeed = 80 # max tip speed [m/s]
    nace.rotor_diameter = 70 # rotor diameter [m]
    nace.rotor_speed = 21.830
    DrivetrainEfficiency = 0.95
    nace.rotor_torque = (nace.machine_rating * 1000 / DrivetrainEfficiency) / (nace.rotor_speed * (pi / 30))
        # rotor torque [Nm] calculated from max / rated rotor speed and machine rating
    nace.rotor_thrust = 324000.
    nace.rotor_mass = 28560. # rotor mass [kg]

    # WindPACT 1.5 MW Tower Variables
    nace.tower_top_diameter = 2.7 # tower top diameter [m]

    print '----- WindPACT 1.5 MW Turbine -----'
    nace.run()

    print 'Nacelle system model results'
    print 'Low speed shaft %8.1f kg %6.2f m Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
          % (nace.lowSpeedShaft.mass , nace.lowSpeedShaft.I[0], nace.lowSpeedShaft.I[1], nace.lowSpeedShaft.I[2], nace.lowSpeedShaft.cm[0], nace.lowSpeedShaft.cm[1], nace.lowSpeedShaft.cm[2])
    # 31257.3 kg
    print 'Main bearings   %8.1f kg ' % (nace.mainBearing.mass + nace.secondBearing.mass)
    # 9731.4 kg
    print 'Gearbox         %8.1f kg %6.2f Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
          % (nace.gearbox.mass, nace.gearbox.I[0], nace.gearbox.I[1], nace.gearbox.I[2], nace.gearbox.cm[0], nace.gearbox.cm[1], nace.gearbox.cm[2] )
    # 30237.6 kg
    #print '     gearbox stage masses: %8.1f kg  %8.1f kg %8.1f kg' % (nace.gearbox.stage_masses[1], nace.gearbox.stage_masses[2], nace.gearbox.stage_masses[3])
    print 'High speed shaft & brakes  %8.1f kg %6.2f Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
          % (nace.highSpeedSide.mass, nace.highSpeedSide.I[0], nace.highSpeedSide.I[1], nace.highSpeedSide.I[2], nace.highSpeedSide.cm[0], nace.highSpeedSide.cm[1], nace.highSpeedSide.cm[2])
    # 1492.4 kg
    print 'Generator       %8.1f kg %6.2f Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
          % (nace.generator.mass, nace.generator.I[0], nace.generator.I[1], nace.generator.I[2], nace.generator.cm[0], nace.generator.cm[1], nace.generator.cm[2])
    # 16699.9 kg
    print 'Variable speed electronics %8.1f kg' % (nace.above_yaw_massAdder.vs_electronics_mass)
    # 0.0 kg
    print 'Overall mainframe %8.1f kg' % (nace.above_yaw_massAdder.mainframe_mass)
    # 96932.9 kg
    print '     Bedplate     %8.1f kg %6.2f Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
         % (nace.bedplate.mass, nace.bedplate.I[0], nace.bedplate.I[1], nace.bedplate.I[2], nace.bedplate.cm[0], nace.bedplate.cm[1], nace.bedplate.cm[2])
    print 'electrical connections  %8.1f kg' % (nace.above_yaw_massAdder.electrical_mass)
    # 0.0 kg
    print 'HVAC system     %8.1f kg' % (nace.above_yaw_massAdder.hvac_mass )
    # 400.0 kg
    print 'Nacelle cover:   %8.1f kg %6.2f m Height %6.2f m Width %6.2f m Length' % (nace.above_yaw_massAdder.cover_mass , nace.above_yaw_massAdder.height, nace.above_yaw_massAdder.width, nace.above_yaw_massAdder.length)
    # 9097.4 kg
    print 'Yaw system      %8.1f kg' % (nace.yawSystem.mass )
    # 11878.2 kg
    print 'Overall nacelle:  %8.1f kg .cm %6.2f %6.2f %6.2f I %6.2f %6.2f %6.2f' % (nace.nacelle_mass, nace.nacelle_cm[0], nace.nacelle_cm[1], nace.nacelle_cm[2], nace.nacelle_I[0], nace.nacelle_I[1], nace.nacelle_I[2]  )
    # 207727.1

    # GRC Drivetrain variables
    nace.drivetrain_design = 1 # geared 3-stage Gearbox with induction generator machine
    nace.machine_rating = 750 # machine rating [kW]
    nace.gear_ratio = 81.491
    nace.gear_configuration = 'epp'
    nace.bevel = 0

    # GRC Rotor Variables
    airdensity = 1.225 # air density [kg / m^3]
    MaxTipSpeed = 8 # max tip speed [m/s]
    nace.rotor_diameter = 48.2 # rotor diameter [m]
    #rotor_speed = MaxTipSpeed / ((rotor_diameter / 2) * (pi / 30)) # max / rated rotor speed [rpm] calculated from max tip speed and rotor diamter
    nace.rotor_speed = 22
    DrivetrainEfficiency = 0.944
    nace.rotor_torque = (nace.machine_rating * 1000 / DrivetrainEfficiency) / (nace.rotor_speed * (pi / 30)) # rotor torque [Nm] calculated from max / rated rotor speed and machine rating
    nace.rotor_thrust = 159000 # based on windpact 750 kW design (GRC information not available)
    nace.rotor_mass = 13200 # rotor mass [kg]

    # Tower Variables
    nace.tower_top_diameter = 2 # tower top diameter [m] - not given

    print '----- GRC 750 kW Turbine -----'
    nace.run()

    print 'Nacelle system model results'
    print 'Low speed shaft %8.1f kg %6.2f m Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
          % (nace.lowSpeedShaft.mass , nace.lowSpeedShaft.I[0], nace.lowSpeedShaft.I[1], nace.lowSpeedShaft.I[2], nace.lowSpeedShaft.cm[0], nace.lowSpeedShaft.cm[1], nace.lowSpeedShaft.cm[2])
    # 31257.3 kg
    print 'Main bearings   %8.1f kg ' % (nace.mainBearing.mass + nace.secondBearing.mass)
    # 9731.4 kg
    print 'Gearbox         %8.1f kg %6.2f Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
          % (nace.gearbox.mass, nace.gearbox.I[0], nace.gearbox.I[1], nace.gearbox.I[2], nace.gearbox.cm[0], nace.gearbox.cm[1], nace.gearbox.cm[2] )
    # 30237.6 kg
    #print '     gearbox stage masses: %8.1f kg  %8.1f kg %8.1f kg' % (nace.gearbox.stage_masses[1], nace.gearbox.stage_masses[2], nace.gearbox.stage_masses[3])
    print 'High speed shaft & brakes  %8.1f kg %6.2f Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
          % (nace.highSpeedSide.mass, nace.highSpeedSide.I[0], nace.highSpeedSide.I[1], nace.highSpeedSide.I[2], nace.highSpeedSide.cm[0], nace.highSpeedSide.cm[1], nace.highSpeedSide.cm[2])
    # 1492.4 kg
    print 'Generator       %8.1f kg %6.2f Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
          % (nace.generator.mass, nace.generator.I[0], nace.generator.I[1], nace.generator.I[2], nace.generator.cm[0], nace.generator.cm[1], nace.generator.cm[2])
    # 16699.9 kg
    print 'Variable speed electronics %8.1f kg' % (nace.above_yaw_massAdder.vs_electronics_mass)
    # 0.0 kg
    print 'Overall mainframe %8.1f kg' % (nace.above_yaw_massAdder.mainframe_mass)
    # 96932.9 kg
    print '     Bedplate     %8.1f kg %6.2f Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
         % (nace.bedplate.mass, nace.bedplate.I[0], nace.bedplate.I[1], nace.bedplate.I[2], nace.bedplate.cm[0], nace.bedplate.cm[1], nace.bedplate.cm[2])
    print 'electrical connections  %8.1f kg' % (nace.above_yaw_massAdder.electrical_mass)
    # 0.0 kg
    print 'HVAC system     %8.1f kg' % (nace.above_yaw_massAdder.hvac_mass )
    # 400.0 kg
    print 'Nacelle cover:   %8.1f kg %6.2f m Height %6.2f m Width %6.2f m Length' % (nace.above_yaw_massAdder.cover_mass , nace.above_yaw_massAdder.height, nace.above_yaw_massAdder.width, nace.above_yaw_massAdder.length)
    # 9097.4 kg
    print 'Yaw system      %8.1f kg' % (nace.yawSystem.mass )
    # 11878.2 kg
    print 'Overall nacelle:  %8.1f kg .cm %6.2f %6.2f %6.2f I %6.2f %6.2f %6.2f' % (nace.nacelle_mass, nace.nacelle_cm[0], nace.nacelle_cm[1], nace.nacelle_cm[2], nace.nacelle_I[0], nace.nacelle_I[1], nace.nacelle_I[2]  )
    # 207727.1

    # Alstom Drivetrain variables
    nace.drivetrain_design = 1 # geared 3-stage Gearbox with induction generator machine
    nace.machine_rating = 3000 # machine rating [kW]
    nace.gear_ratio = 102.19 #
    nace.gear_configuration = 'eep'
    nace.bevel = 0

    # Alstom Rotor Variables
    airdensity = 1.225 # air density [kg / m^3]
    MaxTipSpeed = 80 # max tip speed [m/s]
    nace.rotor_diameter = 100.8 # rotor diameter [m]
    #rotor_speed = MaxTipSpeed / ((rotor_diameter / 2) * (pi / 30)) # max / rated rotor speed [rpm] calculated from max tip speed and rotor diamter
    nace.rotor_speed = 15.1 # based on windpact 3 MW
    DrivetrainEfficiency = 0.95
    nace.rotor_torque = (nace.machine_rating * 1000 / DrivetrainEfficiency) / (nace.rotor_speed * (pi / 30)) # rotor torque [Nm] calculated from max / rated rotor speed and machine rating
    nace.rotor_thrust = 797000. # based on windpact 3.0 MW - Alstom thrust not provided
    nace.rotor_mass = 49498. # rotor mass [kg] - not given - using Windpact 3.0 MW

    # Tower Variables
    nace.tower_top_diameter = 3.5 # tower top diameter [m] - not given

    print '----- Alstom 3.0 MW Turbine -----'
    nace.run()

    print 'Nacelle system model results'
    print 'Low speed shaft %8.1f kg %6.2f m Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
          % (nace.lowSpeedShaft.mass , nace.lowSpeedShaft.I[0], nace.lowSpeedShaft.I[1], nace.lowSpeedShaft.I[2], nace.lowSpeedShaft.cm[0], nace.lowSpeedShaft.cm[1], nace.lowSpeedShaft.cm[2])
    # 31257.3 kg
    print 'Main bearings   %8.1f kg ' % (nace.mainBearing.mass + nace.secondBearing.mass)
    # 9731.4 kg
    print 'Gearbox         %8.1f kg %6.2f Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
          % (nace.gearbox.mass, nace.gearbox.I[0], nace.gearbox.I[1], nace.gearbox.I[2], nace.gearbox.cm[0], nace.gearbox.cm[1], nace.gearbox.cm[2] )
    # 30237.6 kg
    #print '     gearbox stage masses: %8.1f kg  %8.1f kg %8.1f kg' % (nace.gearbox.stage_masses[1], nace.gearbox.stage_masses[2], nace.gearbox.stage_masses[3])
    print 'High speed shaft & brakes  %8.1f kg %6.2f Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
          % (nace.highSpeedSide.mass, nace.highSpeedSide.I[0], nace.highSpeedSide.I[1], nace.highSpeedSide.I[2], nace.highSpeedSide.cm[0], nace.highSpeedSide.cm[1], nace.highSpeedSide.cm[2])
    # 1492.4 kg
    print 'Generator       %8.1f kg %6.2f Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
          % (nace.generator.mass, nace.generator.I[0], nace.generator.I[1], nace.generator.I[2], nace.generator.cm[0], nace.generator.cm[1], nace.generator.cm[2])
    # 16699.9 kg
    print 'Variable speed electronics %8.1f kg' % (nace.above_yaw_massAdder.vs_electronics_mass)
    # 0.0 kg
    print 'Overall mainframe %8.1f kg' % (nace.above_yaw_massAdder.mainframe_mass)
    # 96932.9 kg
    print '     Bedplate     %8.1f kg %6.2f Ixx %6.2f Iyy %6.2f Izz %6.2f CGx %6.2f CGy %6.2f CGz' \
         % (nace.bedplate.mass, nace.bedplate.I[0], nace.bedplate.I[1], nace.bedplate.I[2], nace.bedplate.cm[0], nace.bedplate.cm[1], nace.bedplate.cm[2])
    print 'electrical connections  %8.1f kg' % (nace.above_yaw_massAdder.electrical_mass)
    # 0.0 kg
    print 'HVAC system     %8.1f kg' % (nace.above_yaw_massAdder.hvac_mass )
    # 400.0 kg
    print 'Nacelle cover:   %8.1f kg %6.2f m Height %6.2f m Width %6.2f m Length' % (nace.above_yaw_massAdder.cover_mass , nace.above_yaw_massAdder.height, nace.above_yaw_massAdder.width, nace.above_yaw_massAdder.length)
    # 9097.4 kg
    print 'Yaw system      %8.1f kg' % (nace.yawSystem.mass )
    # 11878.2 kg
    print 'Overall nacelle:  %8.1f kg .cm %6.2f %6.2f %6.2f I %6.2f %6.2f %6.2f' % (nace.nacelle_mass, nace.nacelle_cm[0], nace.nacelle_cm[1], nace.nacelle_cm[2], nace.nacelle_I[0], nace.nacelle_I[1], nace.nacelle_I[2]  )


if __name__ == '__main__':
    ''' Main runs through tests of several drivetrain configurations with known component masses and dimensions '''

    example()

    example2()