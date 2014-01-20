"""
nacelleSE_components.py

Created by Katherine Dykes 2012.
Copyright (c) NREL. All rights reserved.
"""

from openmdao.main.api import Component
from openmdao.main.datatypes.api import Float, Bool, Int, Str, Array
import numpy as np
from math import pi
import algopy

# -------------------------------------------------


class LowSpeedShaft(Component):
    ''' LowSpeedShaft class
          The LowSpeedShaft class is used to represent the low speed shaft component of a wind turbine drivetrain.
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.
    '''

    # variables
    rotor_diameter = Float(iotype='in', units='m', desc='rotor diameter')
    rotor_mass = Float(iotype='in', units='kg', desc='rotor mass')
    rotor_torque = Float(iotype='in', units='N*m', desc='rotor torque at rated power')

    # returns
    design_torque = Float(iotype='out', units='N*m', desc='lss design torque')
    design_bending_load = Float(iotype='out', units='N', desc='lss design bending load')
    length = Float(iotype='out', units='m', desc='lss length')
    diameter = Float(iotype='out', units='m', desc='lss outer diameter')
    mass = Float(0.0, iotype='out', units='kg', desc='overall component mass')
    cm = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system')
    I = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass')

    def __init__(self):
        '''
        Initializes low speed shaft component

        Parameters
        ----------
        rotor_diameter : float
          The wind turbine rotor diameter [m]
        rotor_mass : float
          The wind turbine rotor mass [kg]
        rotor_torque : float
          The input torque from the wind turbine at rated power after accounting for drivetrain losses [N*m]

        Returns
        -------
        design_torque : float
          Design torque for the low speed shaft based on input torque from the rotor at rated speed accounting for drivetrain losses - multiplied by a safety factor
        design_bending_load : float
          Design bending load based on low speed shaft based on rotor mass
        length : float
          Low Speed Shaft length [m]
        diameter : float
          Low Speed shaft outer diameter [m]
        mass : float
          mass of the component [kg]
        cm : array of float
          center of mass of the component (relative to tower top center) [m, m, m]
        I : array of float
          principle moments of inertia for the component (Ixx, iyy, Izz) around its center of mass (relative to tower top center) [kg*m^2, kg*m^2, kg*m^2]
        '''

        super(LowSpeedShaft, self).__init__()

        #controls what happens if derivatives are missing
        self.missing_deriv_policy = 'assume_zero'

    def execute(self):

        # compute masses, dimensions and cost
        ioratio   = 0.100                                    # constant value for inner/outer diameter ratio (should depend on LSS type)
        hollow    = 1/(1.-(ioratio)**4.)                    # hollowness factor based on diameter ratio

        TQsafety       = 3.0                                    # safety factor for design torque applied to rotor torque
        self.design_torque =(TQsafety * self.rotor_torque)            # LSS design torque [Nm]

        lenFact        = 0.03                                   # constant value for length as a function of rotor diameter (should depend on LSS type)
        self.length = (lenFact * self.rotor_diameter )              # LSS shaft length [m]
        maFact         = 5.0                                # moment arm factor from shaft lenght (should depend on shaft type)
        mmtArm    = self.length / maFact            # LSS moment arm [m] - from hub to first main bearing
        BLsafety       = 1.25                                   # saftey factor on bending load
        g              = 9.81                              # gravitational constant [m / s^2]
        self.design_bending_load  = BLsafety * g * self.rotor_mass          # LSS design bending load [N]
        bendMom   = self.design_bending_load * mmtArm       # LSS design bending moment [Nm]

        yieldst        = 371000000.0                             # BS1503-622 yield stress [Pa] (should be adjusted depending on material type)
        endurstsp      = 309000000.0                             # BS1503-625 specimen endurance limit [Pa] (should be adjusted depending on material type)
        endurFact      = 0.23                                    # factor for specimen to component endurance limit
                                                                # (0.75 surface condition * 0.65 size * 0.52 reliability * 1 temperature * 0.91 stress concentration)
        endurst        = endurstsp * endurFact                   # endurance limit [Pa] for LSS
        SOsafety       = 3.25                               # Soderberg Line approach factor for safety
        self.diameter = ((32./pi)*hollow*SOsafety*((self.design_torque / yieldst)**2.+(bendMom/endurst)**2.)**(0.5))**(1./3.)
                                                            # outer diameter [m] computed by Westinghouse Code Formula based on Soderberg Line approach to fatigue design
        inDiam    = self.diameter * ioratio            # inner diameter [m]


        massFact       = 1.25                                    # mass weight factor (should depend on LSS/drivetrain type, currently from windpact modifications to account for flange weight)
        steeldens      = 7860                                    # steel density [kg / m^3]

        self.mass = (massFact*(pi/4.)*(self.diameter**2.-inDiam**2.)*self.length*steeldens)      # mass of LSS [kg]

        # calculate mass properties
        cm = np.array([0.0,0.0,0.0])
        cm[0] = - (0.035 - 0.01) * self.rotor_diameter            # cm based on WindPACT work - halfway between locations of two main bearings
        cm[1] = 0.0
        cm[2] = 0.025 * self.rotor_diameter
        self.cm = cm

        I = np.array([0.0, 0.0, 0.0])
        I[0]  = self.mass * (inDiam ** 2 + self.diameter ** 2) / 8.
        I[1]  = self.mass * (inDiam ** 2 + self.diameter ** 2 + (4. / 3.) * (self.length ** 2)) / 16.
        I[2]  = I[1]
        self.I = I

        # derivatives
        self.d_length_d_rotor_diameter = lenFact
        self.d_torque_d_rotor_torque = TQsafety
        self.d_bending_d_rotor_mass = BLsafety * g

        self.d_diameter_d_rotor_diameter = ((1/6.) * ((32./pi) * hollow*SOsafety)**(1/3.)) * (((self.design_torque/yieldst)**2. + \
                                           (bendMom/endurst)**2.)**(-5./6.)) * (2. * bendMom / (endurst**2.)) * (self.design_bending_load * self.d_length_d_rotor_diameter / maFact)
        self.d_diameter_d_rotor_torque = ((1/6.) * ((32./pi) * hollow * SOsafety)**(1/3.)) * (((self.design_torque/yieldst)**2. + \
                                           (bendMom/endurst)**2.)**(-5/6.)) * (2. * self.design_torque / (yieldst**2.)) * (self.d_torque_d_rotor_torque)
        self.d_diameter_d_rotor_mass = ((1/6.) * ((32./pi) * hollow * SOsafety)**(1/3.)) * (((self.design_torque/yieldst)**2. + \
                                           (bendMom/endurst)**2.)**(-5/6.)) * (2. * bendMom / (endurst**2.)) * (mmtArm * self.d_bending_d_rotor_mass)

        self.d_lss_mass_d_rotor_diameter = massFact * (pi/4) * (1-ioratio**2) * steeldens * (self.diameter**2) * self.d_length_d_rotor_diameter + \
                                    2. * massFact * (pi/4) * (1-ioratio**2) * steeldens * self.length * self.diameter * self.d_diameter_d_rotor_diameter
        self.d_lss_mass_d_rotor_torque = 2. * massFact * (pi/4) * (1-ioratio**2) * steeldens * self.length * self.diameter * self.d_diameter_d_rotor_torque
        self.d_lss_mass_d_rotor_mass =  2. * massFact * (pi/4) * (1-ioratio**2) * steeldens * self.length * self.diameter * self.d_diameter_d_rotor_mass

        self.d_cm_d_rotor_diameter = np.array([-(0.035 - 0.01), 0.0, 0.025])

        self.d_I_d_rotor_diameter = np.array([0.0, 0.0, 0.0])
        self.d_I_d_rotor_diameter[0] = (1/8.) * (1+ ioratio**2.) * self.d_lss_mass_d_rotor_diameter * (self.diameter**2.) + (1/8.) * (1+ioratio**2.) * self.mass * 2. * self.diameter * self.d_diameter_d_rotor_diameter
        self.d_I_d_rotor_diameter[1] = (1./2.) * self.d_I_d_rotor_diameter[0] + (1./16.) * (4./3.) * 2. * self.length * self.d_length_d_rotor_diameter * self.mass + \
                                       (1./16.) * (4./3.) * (self.length**2.) * self.d_lss_mass_d_rotor_diameter
        self.d_I_d_rotor_diameter[2] = self.d_I_d_rotor_diameter[1]

        self.d_I_d_rotor_torque = np.array([0.0, 0.0, 0.0])
        self.d_I_d_rotor_torque[0] = (1/8.) * (1+ ioratio**2.) * self.d_lss_mass_d_rotor_torque * (self.diameter**2.) + (1/8.) * (1+ioratio**2.) * self.mass * 2. * self.diameter * self.d_diameter_d_rotor_torque
        self.d_I_d_rotor_torque[1] = (1/2.) * self.d_I_d_rotor_torque[0] + (1./16.) * (4./3.) * (self.length**2.) * self.d_lss_mass_d_rotor_torque
        self.d_I_d_rotor_torque[2] = self.d_I_d_rotor_torque[1]

        self.d_I_d_rotor_mass = np.array([0.0, 0.0, 0.0])
        self.d_I_d_rotor_mass[0] = (1/8.) * (1+ ioratio**2.) * self.d_lss_mass_d_rotor_mass * (self.diameter**2.) + (1/8.) * (1+ioratio**2.) * self.mass * 2. * self.diameter * self.d_diameter_d_rotor_mass
        self.d_I_d_rotor_mass[1] = (1/2.) * self.d_I_d_rotor_mass[0] + (1./16.) * (4./3.) * (self.length**2.) * self.d_lss_mass_d_rotor_mass
        self.d_I_d_rotor_mass[2] = self.d_I_d_rotor_mass[1]

    def list_deriv_vars(self):

        inputs = ['rotor_diameter', 'rotor_torque', 'rotor_mass']
        outputs = ['length', 'design_torque', 'design_bending_load', 'mass', 'cm', 'I']

        return inputs, outputs

    def provideJ(self):

        # Jacobian
        self.J = np.array([[self.d_length_d_rotor_diameter, 0, 0], \
                           [0, self.d_torque_d_rotor_torque, 0], \
                           [0, 0, self.d_bending_d_rotor_mass], \
                           [self.d_lss_mass_d_rotor_diameter, self.d_lss_mass_d_rotor_torque, self.d_lss_mass_d_rotor_mass], \
                           [self.d_cm_d_rotor_diameter[0], 0, 0], \
                           [self.d_cm_d_rotor_diameter[1], 0, 0], \
                           [self.d_cm_d_rotor_diameter[2], 0, 0], \
                           [self.d_I_d_rotor_diameter[0], self.d_I_d_rotor_torque[0], self.d_I_d_rotor_mass[0]], \
                           [self.d_I_d_rotor_diameter[1], self.d_I_d_rotor_torque[1], self.d_I_d_rotor_mass[1]], \
                           [self.d_I_d_rotor_diameter[2], self.d_I_d_rotor_torque[2], self.d_I_d_rotor_mass[2]]])

        return self.J

#-------------------------------------------------------------------------------

class MainBearing(Component):
    ''' MainBearings class
          The MainBearings class is used to represent the main bearing components of a wind turbine drivetrain. It contains two subcomponents (main bearing and second bearing) which also inherit from the SubComponent class.
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.
    '''
    # variables
    lss_design_torque = Float(iotype='in', units='N*m', desc='lss design torque')
    lss_diameter = Float(iotype='in', units='m', desc='lss outer diameter')
    lss_mass = Float(iotype='in', units='kg', desc='lss mass')
    rotor_speed = Float(iotype='in', units='rpm', desc='rotor speed at rated')
    rotor_diameter = Float(iotype='in', units='m', desc='rotor diameter')

    # returns
    mass = Float(0.0, iotype='out', units='kg', desc='overall component mass')
    cm = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system')
    I = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass')

    def __init__(self):
        ''' Initializes main bearing component

        Parameters
        ----------
        lss_design_torque : float
          Low speed shaft design torque [N*m]
        lss_diameter : float
          Low speed shaft diameter [m]
        lss_mass : float
          Low speed shaft mass [kg]
        rotor_speed : float
          Speed of the rotor at rated power [rpm]
        rotor_diameter : float
          The wind turbine rotor diameter [m]

        Returns
        -------
        mass : float
          mass of the component [kg]
        cm : array of float
          center of mass of the component (relative to tower top center) [m, m, m]
        I : array of float
          principle moments of inertia for the component (Ixx, iyy, Izz) around its center of mass (relative to tower top center) [kg*m^2, kg*m^2, kg*m^2]
        '''

        super(MainBearing, self).__init__()

        #controls what happens if derivatives are missing
        self.missing_deriv_policy = 'assume_zero'

    def execute(self):

        # compute masses, dimensions and cost
        g = 9.81                                           # gravitational constant [m / s^2]
        design1SL = (4.0 / 3.0) * self.lss_design_torque + self.lss_mass * (g / 2.0)
                                                           # front bearing static design load [N] based on default equation (should depend on LSS type)
        design1DL = 2.29 * design1SL * (self.rotor_speed ** 0.3)
                                                           # front bearing dynamic design load [N]

        ratingDL  = 17.96 * ((self.lss_diameter * 1000.0) ** 1.9752)  # basic dynamic load rating for a bearing given inside diameter based on catalogue regression

        massFact  = 0.25                                 # bearing weight factor (should depend on drivetrain type) - using to adjust data closer to cost and scaling model estimates

        if (design1DL < ratingDL):
            b1mass = massFact * (26.13 * (10 ** (-6))) * ((self.lss_diameter * 1000.0) ** 2.77)
                                                           # bearing mass [kg] for single row bearing (based on catalogue data regression)
            h1mass = massFact * (67.44 * (10 ** (-6))) * ((self.lss_diameter * 1000.0) ** 2.64)
                                                           # bearing housing mass [kg] for single row bearing (based on catalogue data regression)
        else:
            b1mass = massFact * 1.7 * (26.13 * (10 ** (-6))) * ((self.lss_diameter * 1000.0) ** 2.77)
                                                          # bearing mass [kg] for double row bearing (based on catalogue data regression)
            h1mass = massFact * 1.5 * (67.44 * (10 ** (-6))) * ((self.lss_diameter * 1000.0) ** 2.64)
                                                          # bearing housing mass [kg] for double row bearing (based on catalogue data regression)

        self.mass = b1mass + h1mass

        # calculate mass properties
        inDiam  = self.lss_diameter
        depth = (inDiam * 1.5)

        cmMB = np.array([0.0,0.0,0.0])
        cmMB = ([- (0.035 * self.rotor_diameter), 0.0, 0.025 * self.rotor_diameter])
        self.cm = cmMB

        b1I0 = (b1mass * inDiam ** 2 ) / 4. + (h1mass * depth ** 2) / 4.
        self.I = ([b1I0, b1I0 / 2., b1I0 / 2.])

        # derivatives
        if design1DL < ratingDL :
            d_bmass_d_lss_diameter = (massFact * (26.13 * (10. ** (-6))) * ((self.lss_diameter * 1000.0) ** 1.77)) * 2.77 * 1000.0
            d_hmass_d_lss_diameter = (massFact * (67.44 * (10. ** (-6))) * ((self.lss_diameter * 1000.0) ** 1.64)) * 2.64 * 1000.0
            self.d_mass_d_lss_diameter = d_bmass_d_lss_diameter + d_hmass_d_lss_diameter
        else :
            d_bmass_d_lss_diameter = (1.7 * massFact * (26.13 * (10. ** (-6))) * ((self.lss_diameter * 1000.0) ** 1.77)) * 2.77 * 1000.0
            d_hmass_d_lss_diameter = (1.5 * massFact * (67.44 * (10. ** (-6))) * ((self.lss_diameter * 1000.0) ** 1.64)) * 2.64 * 1000.0
            self.d_mass_d_lss_diameter = d_bmass_d_lss_diameter + d_hmass_d_lss_diameter

        self.d_cm_d_rotor_diameter = np.array([-0.035, 0.0, 0.025])
        self.d_I_d_lss_diameter = np.array([0.0, 0.0, 0.0])
        self.d_I_d_lss_diameter[0] = d_bmass_d_lss_diameter * inDiam ** 2 / 4. + b1mass * (2. * inDiam) / 4. + \
                                     d_hmass_d_lss_diameter * depth ** 2 / 4. + h1mass * (2. * depth) / 4. * 1.5
        self.d_I_d_lss_diameter[1] = (1/2.) * self.d_I_d_lss_diameter[0]
        self.d_I_d_lss_diameter[2] = (1/2.) * self.d_I_d_lss_diameter[0]

    def list_deriv_vars(self):

        inputs = ['lss_diameter', 'rotor_diameter']
        outputs = ['mass', 'cm', 'I']

        return inputs, outputs

    def provideJ(self):

        # Jacobian
        self.J = np.array([[self.d_mass_d_lss_diameter, 0], \
                           [0, self.d_cm_d_rotor_diameter[0]], \
                           [0, self.d_cm_d_rotor_diameter[1]], \
                           [0, self.d_cm_d_rotor_diameter[2]], \
                           [self.d_I_d_lss_diameter[0], 0], \
                           [self.d_I_d_lss_diameter[1], 0], \
                           [self.d_I_d_lss_diameter[2], 0]])

        return self.J

#-------------------------------------------------------------------------------

class SecondBearing(Component):
    ''' MainBearings class
          The MainBearings class is used to represent the main bearing components of a wind turbine drivetrain. It contains two subcomponents (main bearing and second bearing) which also inherit from the SubComponent class.
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.
    '''
    # variables
    lss_design_torque = Float(iotype='in', units='N*m', desc='lss design torque')
    lss_diameter = Float(iotype='in', units='m', desc='lss outer diameter')
    lss_mass = Float(iotype='in', units='kg', desc='lss mass')
    rotor_speed = Float(iotype='in', units='rpm', desc='rotor speed at rated')
    rotor_diameter = Float(iotype='in', units='m', desc='rotor diameter')

    # returns
    mass = Float(0.0, iotype='out', units='kg', desc='overall component mass')
    cm = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system')
    I = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass')

    def __init__(self):
        ''' Initializes second bearing component

        Parameters
        ----------
        lss_design_torque : float
          Low speed shaft design torque [N*m]
        lss_diameter : float
          Low speed shaft diameter [m]
        lss_mass : float
          Low speed shaft mass [kg]
        rotor_speed : float
          Speed of the rotor at rated power [rpm]
        rotor_diameter : float
          The wind turbine rotor diameter [m]

        Returns
        -------
        mass : float
          mass of the component [kg]
        cm : array of float
          center of mass of the component (relative to tower top center) [m, m, m]
        I : array of float
          principle moments of inertia for the component (Ixx, iyy, Izz) around its center of mass (relative to tower top center) [kg*m^2, kg*m^2, kg*m^2]
        '''

        super(SecondBearing, self).__init__()

        #controls what happens if derivatives are missing
        self.missing_deriv_policy = 'assume_zero'

    def execute(self):

        # compute masses, dimensions and cost
        g = 9.81                                           # gravitational constant [m / s^2]
        design2SL = (1.0 / 3.0) * self.lss_design_torque - self.lss_mass * (g / 2.0)
                                                           # rear bearing static design load [N] based on default equation (should depend on LSS type)
        design2DL = 2.29 * design2SL * (self.rotor_speed ** 0.3)
                                                           # rear bearing dynamic design load [N]
        ratingDL  = 17.96 * ((self.lss_diameter * 1000.0) ** 1.9752)  # basic dynamic load rating for a bearing given inside diameter based on catalogue regression
        massFact  = 0.25                                 # bearing weight factor (should depend on drivetrain type) - using to adjust data closer to cost and scaling model estimates

        if (design2DL < ratingDL):
            b2mass = massFact * (26.13 * (10 ** (-6))) * ((self.lss_diameter * 1000.0) ** 2.77)
                                                           # bearing mass [kg] for single row bearing (based on catalogue data regression)
            h2mass = massFact * (67.44 * (10 ** (-6))) * ((self.lss_diameter * 1000.0) ** 2.64)
                                                           # bearing housing mass [kg] for single row bearing (based on catalogue data regression)
        else:
            b2mass = massFact * 1.7 * (26.13 * (10 ** (-6))) * ((self.lss_diameter * 1000.0) ** 2.77)
                                                          # bearing mass [kg] for double row bearing (based on catalogue data regression)
            h2mass = massFact * 1.5 * (67.44 * (10 ** (-6))) * ((self.lss_diameter * 1000.0) ** 2.64)
                                                          # bearing housing mass [kg] for double row bearing (based on catalogue data regression)

        self.mass = b2mass + h2mass

        # calculate mass properties
        inDiam  = self.lss_diameter
        depth = (inDiam * 1.5)

        cmSB = np.array([0.0,0.0,0.0])
        cmSB = ([- (0.01 * self.rotor_diameter), 0.0, 0.025 * self.rotor_diameter])
        self.cm = cmSB

        b2I0  = (b2mass * inDiam ** 2 ) / 4 + (h2mass * depth ** 2) / 4
        self.I = ([b2I0, b2I0 / 2, b2I0 / 2])

        # derivatives
        if design2DL < ratingDL :
            d_bmass_d_lss_diameter = (massFact * (26.13 * (10. ** (-6))) * ((self.lss_diameter * 1000.0) ** 1.77)) * 2.77 * 1000.0
            d_hmass_d_lss_diameter = (massFact * (67.44 * (10. ** (-6))) * ((self.lss_diameter * 1000.0) ** 1.64)) * 2.64 * 1000.0
            self.d_mass_d_lss_diameter = d_bmass_d_lss_diameter + d_hmass_d_lss_diameter
        else :
            d_bmass_d_lss_diameter = (1.7 * massFact * (26.13 * (10. ** (-6))) * ((self.lss_diameter * 1000.0) ** 1.77)) * 2.77 * 1000.0
            d_hmass_d_lss_diameter = (1.5 * massFact * (67.44 * (10. ** (-6))) * ((self.lss_diameter * 1000.0) ** 1.64)) * 2.64 * 1000.0
            self.d_mass_d_lss_diameter = d_bmass_d_lss_diameter + d_hmass_d_lss_diameter

        self.d_cm_d_rotor_diameter = np.array([-0.01, 0.0, 0.025])
        self.d_I_d_lss_diameter = np.array([0.0, 0.0, 0.0])
        self.d_I_d_lss_diameter[0] = d_bmass_d_lss_diameter * inDiam ** 2 / 4. + b2mass * (2. * inDiam) / 4. + \
                                     d_hmass_d_lss_diameter * depth ** 2 / 4. + h2mass * (2. * depth) / 4. * 1.5
        self.d_I_d_lss_diameter[1] = (1/2.) * self.d_I_d_lss_diameter[0]
        self.d_I_d_lss_diameter[2] = (1/2.) * self.d_I_d_lss_diameter[0]

    def list_deriv_vars(self):

        inputs = ['lss_diameter', 'rotor_diameter']
        outputs = ['mass', 'cm', 'I']

        return inputs, outputs

    def provideJ(self):

        # Jacobian
        self.J = np.array([[self.d_mass_d_lss_diameter, 0], \
                           [0, self.d_cm_d_rotor_diameter[0]], \
                           [0, self.d_cm_d_rotor_diameter[1]], \
                           [0, self.d_cm_d_rotor_diameter[2]], \
                           [self.d_I_d_lss_diameter[0], 0], \
                           [self.d_I_d_lss_diameter[1], 0], \
                           [self.d_I_d_lss_diameter[2], 0]])

        return self.J

#-------------------------------------------------------------------------------

class Gearbox(Component):
    ''' Gearbox class
          The Gearbox class is used to represent the gearbox component of a wind turbine drivetrain.
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.
    '''

    # variables
    rotor_diameter = Float(iotype='in', units='m', desc='rotor diameter')
    rotor_torque = Float(iotype='in', units='N*m', desc='rotor torque at rated power')
    gear_ratio = Float(iotype='in', desc='overall gearbox ratio')

    # parameters
    drivetrain_design = Int(iotype='in', desc='type of gearbox based on drivetrain type: 1 = standard 3-stage gearbox, 2 = single-stage, 3 = multi-gen, 4 = direct drive')
    gear_configuration = Str(iotype='in', desc='string that represents the configuration of the gearbox (stage number and types)')
    bevel = Int(iotype='in', desc='Flag for the presence of a bevel stage - 1 if present, 0 if not')

    # returns
    stage_masses = Array(np.array([None, 0.0, 0.0, 0.0, 0.0]), iotype='out', desc='individual gearbox stage masses')
    mass = Float(0.0, iotype='out', units='kg', desc='overall component mass')
    cm = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system')
    I = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass')

    def __init__(self):
        '''
        Initializes gearbox component

        Parameters
        ----------
        rotor_diameter : float
          The wind turbine rotor diameter [m]
        rotor_torque : float
          The input torque from the wind turbine at rated power after accounting for drivetrain losses [N*m]
        gear_ratio : float
          Overall gear ratio of the gearbox
        drivetrain_design : int
          type of gearbox based on drivetrain type: 1 = standard 3-stage gearbox, 2 = single-stage, 3 = multi-gen, 4 = direct drive
        gear_configuration : string
          string that represents the configuration of the gearbox (stage number and types)
        bevel : int
          Flag for the presence of a bevel stage - 1 if present, 0 if not

        Returns
        -------
        stage_masses : array of float
          individual stage mass of the gearbox [kg]
        mass : float
          mass of the component [kg]
        cm : array of float
          center of mass of the component (relative to tower top center) [m, m, m]
        I : array of float
          principle moments of inertia for the component (Ixx, iyy, Izz) around its center of mass (relative to tower top center) [kg*m^2, kg*m^2, kg*m^2]
        '''

        super(Gearbox, self).__init__()

        #controls what happens if derivatives are missing
        self.missing_deriv_policy = 'assume_zero'

    def execute(self):

        self.mass = self.calc_mass([self.rotor_diameter, self.rotor_torque, self.gear_ratio]) # [self.mass, self.cm[0], self.cm[1], self.cm[2], self.I[0], self.I[1], self.I[2]]

        # calculate mass properties
        cm0   = 0.0
        cm1   = cm0
        cm2   = 0.025 * self.rotor_diameter
        self.cm = np.array([cm0, cm1, cm2])

        length = (0.012 * self.rotor_diameter)
        height = (0.015 * self.rotor_diameter)
        diameter = (0.75 * height)

        I0 = self.mass * (diameter ** 2 ) / 8. + (self.mass / 2.) * (height ** 2) / 8.
        I1 = self.mass * (0.5 * (diameter ** 2) + (2 / 3.) * (length ** 2) + 0.25 * (height ** 2)) / 8.
        I2 = I1
        self.I = np.array([I0, I1, I2])

        # derviatives
        x = algopy.UTPM.init_jacobian([self.rotor_diameter, self.rotor_torque, self.gear_ratio])

        self.d_mass = algopy.UTPM.extract_jacobian(self.calc_mass(x))
        self.d_mass_d_rotor_torque = self.d_mass[1]
        self.d_mass_d_gear_ratio = self.d_mass[2]

        self.d_cm_d_rotor_diameter = np.array([0.0, 0.0, 0.025])

        self.d_I_d_rotor_diameter = np.array([0.0, 0.0, 0.0])
        self.d_I_d_rotor_diameter[0] = (1/8.) * self.mass * (2 * diameter * (0.75 * 0.015) + 0.5 * (2 * height * 0.015))
        self.d_I_d_rotor_diameter[1] = (1/8.) * self.mass * ((0.5 * (2 * diameter) * (0.75 * 0.015)) + \
                                 (2/3.) * (2 * length) * 0.012 + 0.25 * (2 * height) * 0.015)
        self.d_I_d_rotor_diameter[2] = self.d_I_d_rotor_diameter[1]

        self.d_I_d_rotor_torque = np.array([0.0, 0.0, 0.0])
        self.d_I_d_rotor_torque[0] = self.d_mass_d_rotor_torque * ((diameter ** 2 ) / 8.  + 1 / 2. * (height ** 2) / 8.)
        self.d_I_d_rotor_torque[1] = self.d_mass_d_rotor_torque * ((0.5 * (diameter ** 2) + (2 / 3.) * (length ** 2) + 0.25 * (height ** 2)) / 8.)
        self.d_I_d_rotor_torque[2] = self.d_I_d_rotor_torque[1]

        self.d_I_d_gear_ratio = np.array([0.0, 0.0, 0.0])
        self.d_I_d_gear_ratio[0] = self.d_mass_d_gear_ratio * ((diameter ** 2 ) / 8. + (1 / 2.) * (height ** 2) / 8.)
        self.d_I_d_gear_ratio[1] = self.d_mass_d_gear_ratio * ((0.5 * (diameter ** 2) + (2 / 3.) * (length ** 2) + 0.25 * (height ** 2)) / 8.)
        self.d_I_d_gear_ratio[2] = self.d_I_d_gear_ratio[1]

        # TODO: hack for second stage mass calcuation
        self.mass = self.calc_mass([self.rotor_diameter, self.rotor_torque, self.gear_ratio]) # [self.mass, self.cm[0], self.cm[1], self.cm[2], self.I[0], self.I[1], self.I[2]]

    def calc_mass(self, x):

        # inputs
        [rotor_diameter, rotor_torque, gear_ratio] = x

        # compute masses, dimensions and cost
        overallweightFact = 1.00                          # default weight factor 1.0 (should depend on drivetrain design)
        self.stage_masses = [None, 0.0, 0.0, 0.0, 0.0]       # TODO: problem initializing stage_masses and accessing in compute

        # find weight of each stage depending on configuration
        # Gear ratio reduced for each stage based on principle that mixed epicyclic/parallel trains have a final stage ratio set at 1:2.5
        if self.gear_configuration == 'p':
            self.stage_masses[1] = self.__getParallelStageWeight(rotor_torque,gear_ratio)
        if self.gear_configuration == 'e':
            self.stage_masses[1] = self.__getEpicyclicStageWeight(rotor_torque,gear_ratio)
        if self.gear_configuration == 'pp':
            self.stage_masses[1] = self.__getParallelStageWeight(rotor_torque,gear_ratio**0.5)
            self.stage_masses[2] = self.__getParallelStageWeight(rotor_torque,gear_ratio**0.5)
        if self.gear_configuration == 'ep':
            self.stage_masses[1] = self.__getEpicyclicStageWeight(rotor_torque,gear_ratio/2.5)
            self.stage_masses[2] = self.__getParallelStageWeight(rotor_torque,2.5)
        if self.gear_configuration == 'ee':
            self.stage_masses[1] = self.__getEpicyclicStageWeight(rotor_torque,gear_ratio**0.5)
            self.stage_masses[2] = self.__getEpicyclicStageWeight(rotor_torque,gear_ratio**0.5)
        if self.gear_configuration == 'eep':
            U1 = (gear_ratio/3.0)**0.5
            U2 = (gear_ratio/3.0)**0.5
            U3 = 3.0
            self.stage_masses[1] = self.__getEpicyclicStageWeight(rotor_torque,1,U1,U2,U3)  #different than sunderland
            self.stage_masses[2] = self.__getEpicyclicStageWeight(rotor_torque,2,U1,U2,U3)
            self.stage_masses[3] = self.__getParallelStageWeight(rotor_torque,3,U1,U2,U3)
        if self.gear_configuration == 'epp':
            U1 = gear_ratio**0.33*1.4
            U2 = (gear_ratio**0.33/1.18)
            U3 = (gear_ratio**0.33/1.18)
            self.stage_masses[1] = self.__getEpicyclicStageWeight(rotor_torque,1,U1,U2,U3)    # not in sunderland
            self.stage_masses[2] = self.__getParallelStageWeight(rotor_torque,2,U1,U2,U3)
            self.stage_masses[3] = self.__getParallelStageWeight(rotor_torque,3,U1,U2,U3)
        if self.gear_configuration == 'eee':
            self.stage_masses[1] = self.__getEpicyclicStageWeight(rotor_torque,gear_ratio**(0.33))
            self.stage_masses[2] = self.__getEpicyclicStageWeight(rotor_torque,gear_ratio**(0.33))
            self.stage_masses[3] = self.__getEpicyclicStageWeight(rotor_torque,gear_ratio**(0.33))
        if self.gear_configuration == 'ppp':
            self.stage_masses[1] = self.__getParallelStageWeight(rotor_torque,gear_ratio**(0.33))
            self.stage_masses[2] = self.__getParallelStageWeight(rotor_torque,gear_ratio**(0.33))
            self.stage_masses[3] = self.__getParallelStageWeight(rotor_torque,gear_ratio**(0.33))


        if (self.bevel):
            self.stage_masses[4] = 0.0454 * (rotor_torque ** 0.85)

        mass = 0.0
        for i in range(1,4):
            mass += self.stage_masses[i]
        mass     *= overallweightFact

        return mass

    def list_deriv_vars(self):

        inputs = ['rotor_diameter', 'rotor_torque', 'gear_ratio']
        outputs = ['mass', 'cm', 'I']

        return inputs, outputs

    def provideJ(self):

        # Jacobian
        self.J = np.array([[0, self.d_mass_d_rotor_torque, self.d_mass_d_gear_ratio], \
                           [self.d_cm_d_rotor_diameter[0], 0, 0], \
                           [self.d_cm_d_rotor_diameter[1], 0, 0], \
                           [self.d_cm_d_rotor_diameter[2], 0, 0], \
                           [self.d_I_d_rotor_diameter[0], self.d_I_d_rotor_torque[0], self.d_I_d_gear_ratio[0]], \
                           [self.d_I_d_rotor_diameter[1], self.d_I_d_rotor_torque[1], self.d_I_d_gear_ratio[1]], \
                           [self.d_I_d_rotor_diameter[2], self.d_I_d_rotor_torque[2], self.d_I_d_gear_ratio[2]]])

        return self.J

    def __getParallelStageWeight(self,rotor_torque,stage,stageRatio1,stageRatio2,stageRatio3):

        '''
          This method calculates the stage weight for a parallel stage in a gearbox based on the input torque, stage number, and stage ratio for each individual stage.
        '''

        serviceFact     = 1.00                                # default service factor for a gear stage is 1.75 based on full span VP (should depend on control type)
        applicationFact = 0.4                             # application factor ???
        stageweightFact = 8.029 #/2                           # stage weight factor applied to each Gearbox stage

        if (rotor_torque * serviceFact) < 200000.0:       # design factor for design and manufacture of Gearbox
            designFact = 925.0
        elif (rotor_torque * serviceFact) < 700000.0:
            designFact = 1000.0
        else:
            designFact = 1100.0                            # TODO: should be an exception for all 2 stage Gearboxes to have designFact = 1000

        if stage == 1:
            Qr         = rotor_torque
            stageRatio = stageRatio1
        elif stage == 2:
            Qr         = rotor_torque/stageRatio1
            stageRatio = stageRatio2
        elif stage == 3:
            Qr         = rotor_torque/(stageRatio1*stageRatio2)
            stageRatio = stageRatio3

        gearFact = applicationFact / designFact          # Gearbox factor for design, manufacture and application of Gearbox

        gearweightFact = 1 + (1 / stageRatio) + stageRatio + (stageRatio ** 2)
                                                         # Gearbox weight factor for relationship of stage ratio required and relative stage volume

        stageWeight = stageweightFact * Qr * serviceFact * gearFact * gearweightFact
                                                         # forumula for parallel gearstage weight based on sunderland model

        return stageWeight

    def __getEpicyclicStageWeight(self,rotor_torque,stage,stageRatio1,stageRatio2,stageRatio3):
        '''
          This method calculates the stage weight for a epicyclic stage in a gearbox based on the input torque, stage number, and stage ratio for each individual stage
        '''

        serviceFact     = 1.00                                # default service factor for a gear stage is 1.75 based on full span VP (should depend on control type)
        applicationFact = 0.4                             # application factor ???
        stageweightFact = 8.029/12                          # stage weight factor applied to each Gearbox stage
        OptWheels       = 3.0                                    # default optional wheels (should depend on stage design)

        if (rotor_torque * serviceFact) < 200000.0:       # design factor for design and manufacture of Gearbox
            designFact = 850.0
        elif (rotor_torque * serviceFact) < 700000.0:
            designFact = 950.0
        else:
            designFact = 1100.0

        if stage == 1:
            Qr         = rotor_torque
            stageRatio = stageRatio1
        elif stage == 2:
            Qr         = rotor_torque/stageRatio1
            stageRatio = stageRatio2
        elif stage == 3:
            Qr         = rotor_torque/(stageRatio1*stageRatio2)
            stageRatio = stageRatio3

        gearFact = applicationFact / designFact          # Gearbox factor for design, manufacture and application of Gearbox

        sunwheelratio  = (stageRatio / 2.0) - 1             # sun wheel ratio for epicyclic Gearbox stage based on stage ratio
        gearweightFact = (1 / OptWheels) + (1 / (OptWheels * sunwheelratio)) + sunwheelratio + \
                         ((1 + sunwheelratio) / OptWheels) * ((stageRatio - 1.) ** 2)
                                                         # Gearbox weight factor for relationship of stage ratio required and relative stage volume

        stageWeight    = stageweightFact * Qr * serviceFact * gearFact * gearweightFact
                                                         # forumula for epicyclic gearstage weight based on sunderland model

        return stageWeight

#-------------------------------------------------------------------------------

class HighSpeedSide(Component):
    '''
    HighSpeedShaft class
          The HighSpeedShaft class is used to represent the high speed shaft and mechanical brake components of a wind turbine drivetrain.
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.
    '''

    # variables
    rotor_diameter = Float(iotype='in', units='m', desc='rotor diameter')
    rotor_torque = Float(iotype='in', units='N*m', desc='rotor torque at rated power')
    gear_ratio = Float(iotype='in', desc='overall gearbox ratio')
    lss_diameter = Float(iotype='in', units='m', desc='low speed shaft outer diameter')

    # returns
    mass = Float(0.0, iotype='out', units='kg', desc='overall component mass')
    cm = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system')
    I = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass')

    def __init__(self):
        '''
        Initializes high speed side component

        Parameters
        ----------
        rotor_diameter : float
          The wind turbine rotor diameter [m]
        rotor_torque : float
          The input torque from the wind turbine at rated power after accounting for drivetrain losses [N*m]
        gear_ratio : float
          Overall gear ratio of the gearbox
        lss_diameter : float
          Low speed shaft diameter [m]

        Returns
        -------
        mass : float
          mass of the component [kg]
        cm : array of float
          center of mass of the component (relative to tower top center) [m, m, m]
        I : array of float
          principle moments of inertia for the component (Ixx, iyy, Izz) around its center of mass (relative to tower top center) [kg*m^2, kg*m^2, kg*m^2]
        '''

        super(HighSpeedSide, self).__init__()

        #controls what happens if derivatives are missing
        self.missing_deriv_policy = 'assume_zero'

    def execute(self):

        # compute masses, dimensions and cost
        design_torque = self.rotor_torque / self.gear_ratio               # design torque [Nm] based on rotor torque and Gearbox ratio
        massFact = 0.025                                 # mass matching factor default value
        highSpeedShaftMass = (massFact * design_torque)

        mechBrakeMass = (0.5 * highSpeedShaftMass)      # relationship derived from HSS multiplier for University of Sunderland model compared to NREL CSM for 750 kW and 1.5 MW turbines

        self.mass = (mechBrakeMass + highSpeedShaftMass)

        # calculate mass properties
        cm = np.array([0.0,0.0,0.0])
        cm[0]   = 0.5 * (0.0125 * self.rotor_diameter)
        cm[1]   = 0.0
        cm[2]   = 0.025 * self.rotor_diameter
        self.cm = cm

        diameter = (1.5 * self.lss_diameter)                     # based on WindPACT relationships for full HSS / mechanical brake assembly
        length = (0.025)
        matlDensity = 7850. # material density kg/m^3

        I = np.array([0.0, 0.0, 0.0])
        I[0]    = 0.25 * length * 3.14159 * matlDensity * (diameter ** 2) * (self.gear_ratio**2) * (diameter ** 2) / 8.
        I[1]    = self.mass * ((3/4.) * (diameter ** 2) + (length ** 2)) / 12.
        I[2]    = I[1]
        self.I = I

        # derivatives
        self.d_mass_d_rotor_torque = 1.5 * massFact / self.gear_ratio
        self.d_mass_d_gear_ratio = 1.5 * massFact * self.rotor_torque * (-1 / (self.gear_ratio**2))

        self.d_cm_d_rotor_diameter = np.array([0.5 * 0.0125, 0.0, 0.025])

        self.d_I_d_lss_diameter = np.array([0.0, 0.0, 0.0])
        self.d_I_d_lss_diameter[0] = 4. * 1.5 * (0.25 * length * 3.14159 * matlDensity * (self.gear_ratio**2) * (diameter ** 3) / 8.)
        self.d_I_d_lss_diameter[1] = 2. * 1.5 * (self.mass * ((3/4.) * diameter / 12.))
        self.d_I_d_lss_diameter[2] = self.d_I_d_lss_diameter[1]

        self.d_I_d_gear_ratio = np.array([0.0, 0.0, 0.0])
        self.d_I_d_gear_ratio[0] = 2. * (0.25 * length * 3.14159 * matlDensity * (diameter ** 2) * (self.gear_ratio) * (diameter ** 2) / 8.)
        self.d_I_d_gear_ratio[1] = self.d_mass_d_gear_ratio * ((3/4.) * (diameter ** 2) + (length ** 2)) / 12.
        self.d_I_d_gear_ratio[2] = self.d_I_d_gear_ratio[1]

        self.d_I_d_rotor_torque = np.array([0.0, 0.0, 0.0])
        self.d_I_d_rotor_torque[1] = self.d_mass_d_rotor_torque * ((3/4.) * (diameter ** 2) + (length ** 2)) / 12.
        self.d_I_d_rotor_torque[2] = self.d_I_d_rotor_torque[1]

    def list_deriv_vars(self):

        inputs = ['rotor_diameter','lss_diameter', 'rotor_torque', 'gear_ratio']
        outputs = ['mass', 'cm', 'I']

        return inputs, outputs

    def provideJ(self):

        # Jacobian
        self.J = np.array([[0, 0, self.d_mass_d_rotor_torque, self.d_mass_d_gear_ratio], \
                           [self.d_cm_d_rotor_diameter[0], 0, 0, 0], \
                           [self.d_cm_d_rotor_diameter[1], 0, 0, 0], \
                           [self.d_cm_d_rotor_diameter[2], 0, 0, 0], \
                           [0, self.d_I_d_lss_diameter[0], self.d_I_d_rotor_torque[0], self.d_I_d_gear_ratio[0]], \
                           [0, self.d_I_d_lss_diameter[1], self.d_I_d_rotor_torque[1], self.d_I_d_gear_ratio[1]], \
                           [0, self.d_I_d_lss_diameter[2], self.d_I_d_rotor_torque[2], self.d_I_d_gear_ratio[2]]])

        return self.J


#-------------------------------------------------------------------------------

class Generator(Component):
    '''Generator class
          The Generator class is used to represent the generator of a wind turbine drivetrain.
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.
    '''

    # variables
    rotor_diameter = Float(iotype='in', units='m', desc='rotor diameter')
    machine_rating = Float(iotype='in', units='kW', desc='machine rating of generator')
    gear_ratio = Float(iotype='in', desc='overall gearbox ratio')

    # parameters
    drivetrain_design = Int(iotype='in', desc='type of gearbox based on drivetrain type: 1 = standard 3-stage gearbox, 2 = single-stage, 3 = multi-gen, 4 = direct drive')

    # returns
    mass = Float(0.0, iotype='out', units='kg', desc='overall component mass')
    cm = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system')
    I = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass')

    def __init__(self):
        '''
        Initializes generator component

        Parameters
        ----------
        rotor_diameter : float
          The wind turbine rotor diameter [m]
        machine_rating : float
          Turbine machine rating [kW]
        gear_ratio : float
          Overall gear ratio of the gearbox
        drivetrain_design : int
          type of gearbox based on drivetrain type: 1 = standard 3-stage gearbox, 2 = single-stage, 3 = multi-gen, 4 = direct drive

        Returns
        -------
        mass : float
          mass of the component [kg]
        cm : array of float
          center of mass of the component (relative to tower top center) [m, m, m]
        I : array of float
          principle moments of inertia for the component (Ixx, iyy, Izz) around its center of mass (relative to tower top center) [kg*m^2, kg*m^2, kg*m^2]
        '''

        super(Generator, self).__init__()

        #controls what happens if derivatives are missing
        self.missing_deriv_policy = 'assume_zero'

    def execute(self):

        massCoeff = [None, 6.4737, 10.51 ,  5.34  , 37.68  ]
        massExp   = [None, 0.9223, 0.9223,  0.9223, 1      ]

        CalcRPM    = 80 / (self.rotor_diameter*0.5*pi/30)
        CalcTorque = (self.machine_rating*1.1) / (CalcRPM * pi/30)

        if (self.drivetrain_design < 4):
            self.mass = (massCoeff[self.drivetrain_design] * self.machine_rating ** massExp[self.drivetrain_design])
        else:  # direct drive
            self.mass = (massCoeff[self.drivetrain_design] * CalcTorque ** massExp[self.drivetrain_design])

        # calculate mass properties
        cm = np.array([0.0,0.0,0.0])
        cm[0]  = 0.0125 * self.rotor_diameter
        cm[1]  = 0.0
        cm[2]  = 0.025 * self.rotor_diameter
        self.cm = cm

        length = (1.6 * 0.015 * self.rotor_diameter)
        depth = (0.015 * self.rotor_diameter)
        width = (0.5 * depth)

        I = np.array([0.0, 0.0, 0.0])
        I[0]   = ((4.86 * (10. ** (-5))) * (self.rotor_diameter ** 5.333)) + (((2./3.) * self.mass) * (depth ** 2 + width ** 2) / 8.)
        I[1]   = (I[0] / 2.) / (self.gear_ratio ** 2) + ((1./3.) * self.mass * (length ** 2) / 12.) + (((2. / 3.) * self.mass) * \
                   (depth ** 2. + width ** 2. + (4./3.) * (length ** 2.)) / 16. )
        I[2]   = I[1]
        self.I = I

        # derivatives
        if (self.drivetrain_design < 4):
            self.d_mass_d_rotor_diameter = 0.0
        else:  # direct drive
            self.d_mass_d_rotor_diameter = massExp[self.drivetrain_design] * (massCoeff[self.drivetrain_design] * CalcTorque ** (massExp[self.drivetrain_design] - 1)) * (self.machine_rating * 1.1 * 0.5 / 80)

        if (self.drivetrain_design < 4):
            self.d_mass_d_machine_rating = massExp[self.drivetrain_design] * (massCoeff[self.drivetrain_design] * self.machine_rating ** (massExp[self.drivetrain_design]-1))
        else:  # direct drive
            self.d_mass_d_machine_rating = massExp[self.drivetrain_design] * (massCoeff[self.drivetrain_design] * CalcTorque ** (massExp[self.drivetrain_design] - 1)) * (self.rotor_diameter * 1.1 * 0.5 / 80)

        self.d_cm_d_rotor_diameter = np.array([0.0125, 0.0, 0.025])

        self.d_I_d_rotor_diameter = np.array([0.0, 0.0, 0.0])
        self.d_I_d_rotor_diameter[0] = ((4.86*(10.**(-5)))*(self.rotor_diameter**4.333)) * 5.333 + \
                                (1./8.) * (2./3.) * self.d_mass_d_rotor_diameter * (depth ** 2 + width ** 2) + \
                                (1./8.) * (2./3.) * self.mass * (2.*depth*0.015 + 2.*width*0.5*0.015)
        self.d_I_d_rotor_diameter[1] = (1./(2.*self.gear_ratio**2))*(self.d_I_d_rotor_diameter[0]) + \
                                 self.d_mass_d_rotor_diameter * (((1./3.) * (length ** 2) / 12.) + ((2. / 3.) * (depth ** 2 + width ** 2 + (4./3.) * (length ** 2)) / 16. )) + \
                                 self.mass * ((1./3.) * (1./12.) * (2. * length * 1.6 * 0.015) + (2./3.) * (1./16.) * (2.*depth*0.015 + 2.*width*0.5*0.015 + (4./3.)*2.*length*1.6*0.015))
        self.d_I_d_rotor_diameter[2] = self.d_I_d_rotor_diameter[1]

        self.d_I_d_machine_rating = np.array([0.0, 0.0, 0.0])
        self.d_I_d_machine_rating[0] = (1./8.) * (2./3.) * self.d_mass_d_machine_rating * (depth ** 2 + width ** 2)
        self.d_I_d_machine_rating[1] = (1/(2.*self.gear_ratio**2))*self.d_I_d_machine_rating[0] + \
                                       ((1/3.) * self.d_mass_d_machine_rating * (length ** 2) / 12.) + \
                                       (((2 / 3.) * self.d_mass_d_machine_rating) * (depth ** 2 + width ** 2 + (4/3.) * (length ** 2)) / 16. )
        self.d_I_d_machine_rating[2] = self.d_I_d_machine_rating[1]

        self.d_I_d_gear_ratio = np.array([0.0, 0.0, 0.0])
        self.d_I_d_gear_ratio[1] = (1/2.) * self.I[0] * (-2.) * (self.gear_ratio**(-3))
        self.d_I_d_gear_ratio[2] = self.d_I_d_gear_ratio[1]

    def list_deriv_vars(self):

        inputs = ['rotor_diameter','machine_rating', 'gear_ratio']
        outputs = ['mass', 'cm', 'I']

        return inputs, outputs

    def provideJ(self):

        # Jacobian
        self.J = np.array([[self.d_mass_d_rotor_diameter, self.d_mass_d_machine_rating, 0], \
                           [self.d_cm_d_rotor_diameter[0], 0, 0], \
                           [self.d_cm_d_rotor_diameter[1], 0, 0], \
                           [self.d_cm_d_rotor_diameter[2], 0, 0], \
                           [self.d_I_d_rotor_diameter[0], self.d_I_d_machine_rating[0], self.d_I_d_gear_ratio[0]], \
                           [self.d_I_d_rotor_diameter[1], self.d_I_d_machine_rating[1], self.d_I_d_gear_ratio[1]], \
                           [self.d_I_d_rotor_diameter[2], self.d_I_d_machine_rating[2], self.d_I_d_gear_ratio[2]]])

        return self.J

#-------------------------------------------------------------------------------

class Bedplate(Component):
    ''' Bedplate class
          The Bedplate class is used to represent the bedplate of a wind turbine drivetrain.
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.
    '''
    # variables
    rotor_diameter = Float(iotype='in', units='m', desc='rotor diameter')
    rotor_thrust = Float(iotype='in', units='N', desc='maximum rotor thrust')
    rotor_mass = Float(iotype='in', units='kg', desc='rotor overall mass')
    rotor_torque = Float(iotype='in', units='N*m', desc='rotor torque at rated power')
    tower_top_diameter = Float(iotype='in', units='m', desc='tower top diameter')

    # parameters
    drivetrain_design = Int(iotype='in', desc='type of gearbox based on drivetrain type: 1 = standard 3-stage gearbox, 2 = single-stage, 3 = multi-gen, 4 = direct drive')

    # returns
    mass = Float(0.0, iotype='out', units='kg', desc='overall component mass')
    cm = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system')
    I = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass')
    length = Float(iotype='out', units='m', desc='length of bedplate')
    width = Float(iotype='out', units='m', desc='width of bedplate')

    def __init__(self):
        '''
        Initializes bedplate component

        Parameters
        ----------
        rotor_diameter : float
          The wind turbine rotor diameter [m]
        rotor_thrust : float
          Maximum thrust from the rotor applied to the drivetrain under extreme conditions [N]
        rotor_mass : float
          Mass of the rotor [kg]
        rotor_torque : float
          The input torque from the wind turbine at rated power after accounting for drivetrain losses [N*m]
        tower_top_diameter : float
          Diameter of the turbine tower top [m]
        drivetrain_design : int
          type of gearbox based on drivetrain type: 1 = standard 3-stage gearbox, 2 = single-stage, 3 = multi-gen, 4 = direct drive

        Returns
        -------
        mass : float
          mass of the component [kg]
        cm : array of float
          center of mass of the component (relative to tower top center) [m, m, m]
        I : array of float
          principle moments of inertia for the component (Ixx, iyy, Izz) around its center of mass (relative to tower top center) [kg*m^2, kg*m^2, kg*m^2]
        length : float
          Bedplate critical length [m]
        width : float
          Bedplate critical width [m]
        '''

        super(Bedplate, self).__init__()

        #controls what happens if derivatives are missing
        self.missing_deriv_policy = 'assume_zero'

    def execute(self):

        # compute masses, dimensions and cost
        # bedplate sizing based on superposition of loads for rotor torque, thurst, weight         #TODO: only handles bedplate for a traditional drivetrain configuration
        bedplateWeightFact = 2.86                                   # toruqe weight factor for bedplate (should depend on drivetrain, bedplate type)

        torqueweightCoeff = 0.00368                   # regression coefficient multiplier for bedplate weight based on rotor torque
        MassFromTorque    = bedplateWeightFact * (torqueweightCoeff * self.rotor_torque)

        thrustweightCoeff = 0.00158                                 # regression coefficient multiplier for bedplate weight based on rotor thrust
        MassFromThrust    = bedplateWeightFact * (thrustweightCoeff * (self.rotor_thrust * self.tower_top_diameter))

        rotorweightCoeff    = 0.015                                    # regression coefficient multiplier for bedplate weight based on rotor weight
        MassFromRotorWeight = bedplateWeightFact * (rotorweightCoeff * (self.rotor_mass * self.tower_top_diameter))

        # additional weight ascribed to bedplate area
        BPlengthFact    = 1.5874                                       # bedplate length factor (should depend on drivetrain, bedplate type)
        nacellevolFact  = 0.052                                      # nacelle volume factor (should depend on drivetrain, bedplate type)
        self.length = (BPlengthFact * nacellevolFact * self.rotor_diameter)     # bedplate length [m] calculated as a function of rotor diameter
        self.width = (self.length / 2.0)                              # bedplate width [m] assumed to be half of bedplate length
        area       = self.length * self.width                        # bedplate area [m^2]
        height = ((2 / 3) * self.length)                         # bedplate height [m] calculated based on cladding area
        areaweightCoeff = 100                                       # regression coefficient multiplier for bedplate weight based on bedplate area
        MassFromArea    = bedplateWeightFact * (areaweightCoeff * area)

        # total mass is calculated based on adding masses attributed to rotor torque, thrust, weight and bedplate area
        TotalMass = MassFromTorque + MassFromThrust + MassFromRotorWeight + MassFromArea

        # for single-stage and multi-generator - bedplate mass based on regresstion to rotor diameter
        # for geared and direct drive - bedplate mass based on total mass as calculated above
        massCoeff    = [None,22448,1.29490,1.72080,22448 ]
        massExp      = [None,    0,1.9525, 1.9525 ,    0 ]
        massCoeff[1] = TotalMass
        ddweightfact = 0.55                                         # direct drive bedplate weight assumed to be 55% of modular geared type
        massCoeff[4] = TotalMass * ddweightfact

        self.mass = (massCoeff[self.drivetrain_design] * self.rotor_diameter ** massExp[self.drivetrain_design] )

        # calculate mass properties
        cm = np.array([0.0,0.0,0.0])
        cm[0] = cm[1] = 0.0
        cm[2] = 0.0122 * self.rotor_diameter                             # half distance from shaft to yaw axis
        self.cm = cm

        depth = (self.length / 2.0)

        I = np.array([0.0, 0.0, 0.0])
        I[0]  = self.mass * (self.width ** 2 + depth ** 2) / 8.
        I[1]  = self.mass * (depth ** 2 + self.width ** 2 + (4/3.) * self.length ** 2) / 16.
        I[2]  = I[1]
        self.I = I

        # derivatives
        self.d_length_d_rotor_diameter = BPlengthFact * nacellevolFact
        self.d_width_d_rotor_diameter = (1/2.0) * self.d_length_d_rotor_diameter
        self.d_depth_d_rotor_diameter = (1/2.0) * self.d_length_d_rotor_diameter

        self.d_mass_d_rotor_diameter = bedplateWeightFact * areaweightCoeff * ((BPlengthFact * nacellevolFact)**2) * self.rotor_diameter
        self.d_mass_d_rotor_thrust = bedplateWeightFact * (thrustweightCoeff * (self.tower_top_diameter))
        self.d_mass_d_rotor_torque = bedplateWeightFact * torqueweightCoeff
        self.d_mass_d_rotor_mass = bedplateWeightFact * (rotorweightCoeff * (self.tower_top_diameter))
        self.d_mass_d_tower_top_diameter = bedplateWeightFact * (thrustweightCoeff * (self.rotor_thrust)) + bedplateWeightFact * (rotorweightCoeff * (self.rotor_mass))

        self.d_cm_d_rotor_diameter = np.array([0.0, 0.0, 0.0122])

        self.d_I_d_rotor_diameter = np.array([0.0, 0.0, 0.0])
        self.d_I_d_rotor_diameter[0] = (1/8.) * (self.d_mass_d_rotor_diameter)*(self.width**2 + depth**2) + (1/8.) * self.mass * (2.*self.width*self.d_width_d_rotor_diameter + 2.*depth*self.d_depth_d_rotor_diameter)
        self.d_I_d_rotor_diameter[1] = (1/16.) * (self.d_mass_d_rotor_diameter)*(self.width**2 + depth**2 + (4/3.) * self.length**2) + (1/16.) * self.mass * (2.*self.width*self.d_width_d_rotor_diameter + 2.*depth*self.d_depth_d_rotor_diameter + (4/3.)*2.*self.length*self.d_length_d_rotor_diameter)
        self.d_I_d_rotor_diameter[2] = self.d_I_d_rotor_diameter[1]

        self.d_I_d_rotor_thrust = np.array([0.0, 0.0, 0.0])
        self.d_I_d_rotor_thrust[0] = (1/8.) * (self.d_mass_d_rotor_thrust)*(self.width**2 + depth**2)
        self.d_I_d_rotor_thrust[1] = (1/16.) * (self.d_mass_d_rotor_thrust)*(self.width**2 + depth**2 + (4/3.) * self.length**2)
        self.d_I_d_rotor_thrust[2] = self.d_I_d_rotor_thrust[1]

        self.d_I_d_rotor_torque = np.array([0.0, 0.0, 0.0])
        self.d_I_d_rotor_torque[0] = (1/8.) * (self.d_mass_d_rotor_torque)*(self.width**2 + depth**2)
        self.d_I_d_rotor_torque[1] = (1/16.) * (self.d_mass_d_rotor_torque)*(self.width**2 + depth**2 + (4/3.) * self.length**2)
        self.d_I_d_rotor_torque[2] = self.d_I_d_rotor_torque[1]

        self.d_I_d_rotor_mass = np.array([0.0, 0.0, 0.0])
        self.d_I_d_rotor_mass[0] = (1/8.) * (self.d_mass_d_rotor_mass)*(self.width**2 + depth**2)
        self.d_I_d_rotor_mass[1] = (1/16.) * (self.d_mass_d_rotor_mass)*(self.width**2 + depth**2 + (4/3.) * self.length**2)
        self.d_I_d_rotor_mass[2] = self.d_I_d_rotor_mass[1]

        self.d_I_d_tower_top_diameter = np.array([0.0, 0.0, 0.0])
        self.d_I_d_tower_top_diameter[0] = (1/8.) * (self.d_mass_d_tower_top_diameter)*(self.width**2 + depth**2)
        self.d_I_d_tower_top_diameter[1] = (1/16.) * (self.d_mass_d_tower_top_diameter)*(self.width**2 + depth**2 + (4/3.) * self.length**2)
        self.d_I_d_tower_top_diameter[2] = self.d_I_d_tower_top_diameter[1]

    def list_deriv_vars(self):

        inputs = ['rotor_diameter','rotor_thrust', 'rotor_torque', 'rotor_mass', 'tower_top_diameter']
        outputs = ['mass', 'cm', 'I', 'length', 'width']

        return inputs, outputs

    def provideJ(self):

        # Jacobian
        self.J = np.array([[self.d_mass_d_rotor_diameter, self.d_mass_d_rotor_thrust, self.d_mass_d_rotor_torque, self.d_mass_d_rotor_mass, self.d_mass_d_tower_top_diameter], \
                           [self.d_cm_d_rotor_diameter[0], 0, 0, 0, 0], \
                           [self.d_cm_d_rotor_diameter[1], 0, 0, 0, 0], \
                           [self.d_cm_d_rotor_diameter[2], 0, 0, 0, 0], \
                           [self.d_I_d_rotor_diameter[0], self.d_I_d_rotor_thrust[0], self.d_I_d_rotor_torque[0], self.d_I_d_rotor_mass[0], self.d_I_d_tower_top_diameter[0]], \
                           [self.d_I_d_rotor_diameter[1], self.d_I_d_rotor_thrust[1], self.d_I_d_rotor_torque[1], self.d_I_d_rotor_mass[1], self.d_I_d_tower_top_diameter[1]], \
                           [self.d_I_d_rotor_diameter[2], self.d_I_d_rotor_thrust[2], self.d_I_d_rotor_torque[2], self.d_I_d_rotor_mass[2], self.d_I_d_tower_top_diameter[2]], \
                           [self.d_length_d_rotor_diameter, 0, 0, 0, 0], \
                           [self.d_width_d_rotor_diameter, 0, 0, 0, 0]])

        return self.J


#-------------------------------------------------------------------------------

class AboveYawMassAdder(Component):

    # variables
    machine_rating = Float(iotype = 'in', units='kW', desc='machine rating')
    lss_mass = Float(iotype = 'in', units='kg', desc='component mass')
    main_bearing_mass = Float(iotype = 'in', units='kg', desc='component mass')
    second_bearing_mass = Float(iotype = 'in', units='kg', desc='component mass')
    gearbox_mass = Float(iotype = 'in', units='kg', desc='component mass')
    hss_mass = Float(iotype = 'in', units='kg', desc='component mass')
    generator_mass = Float(iotype = 'in', units='kg', desc='component mass')
    bedplate_mass = Float(iotype = 'in', units='kg', desc='component mass')
    bedplate_length = Float(iotype = 'in', units='m', desc='component length')
    bedplate_width = Float(iotype = 'in', units='m', desc='component width')

    # parameters
    crane = Bool(iotype='in', desc='flag for presence of crane')

    # returns
    electrical_mass = Float(iotype = 'out', units='kg', desc='component mass')
    vs_electronics_mass = Float(iotype = 'out', units='kg', desc='component mass')
    hvac_mass = Float(iotype = 'out', units='kg', desc='component mass')
    controls_mass = Float(iotype = 'out', units='kg', desc='component mass')
    platforms_mass = Float(iotype = 'out', units='kg', desc='component mass')
    crane_mass = Float(iotype = 'out', units='kg', desc='component mass')
    mainframe_mass = Float(iotype = 'out', units='kg', desc='component mass')
    cover_mass = Float(iotype = 'out', units='kg', desc='component mass')
    above_yaw_mass = Float(iotype = 'out', units='kg', desc='total mass above yaw system')
    length = Float(iotype = 'out', units='m', desc='component length')
    width = Float(iotype = 'out', units='m', desc='component width')
    height = Float(iotype = 'out', units='m', desc='component height')

    def __init__(self):
        ''' Initialize above yaw mass adder component

        Parameters
        ----------
        machine_rating : float
          machine rating [kW]
        lss_mass : float
          low speed shaft component mass [kg]
        main_bearing_mass : float
          main bearing component mass [kg]
        second_bearing_mass : float
          second bearing component mass [kg]
        gearbox_mass : float
          gearbox component mass [kg]
        hss_mass : float
          high speed side component mass [kg]
        generator_mass : float
          generator component mass [kg]
        bedplate_mass : float
          bedplate component mass [kg]
        bedplate_length : float
          bedplate component length [m]
        bedplate_width : float
          bedplate component width [m]
        crane : boolean
          flag for presence of crane

        Returns
        -------
        electrical_mass : float
          electrical connections component mass [kg]
        vs_electronics_mass : float
          variable speed electrics mass [kg]
        hvac_mass : float
          HVAC component mass [kg]
        controls_mass : float
          controls component mass [kg]
        platforms_mass : float
          nacelle platforms component mass [kg]
        crane_mass : float
          crane component mass [kg]
        mainframe_mass : float
          mainframe component mass [kg]
        cover_mass : float
          nacelle cover component mass [kg]
        above_yaw_mass : float
          total mass above yaw system [kg]
        length : float
          nacelle component length [m]
        width : float
          nacelle component width [m]
        height : float
          nacelle component height [m]
        '''

        super(AboveYawMassAdder, self).__init__()

        #controls what happens if derivatives are missing
        self.missing_deriv_policy = 'assume_zero'

    def execute(self):

        # electronic systems, hydraulics and controls
        self.electrical_mass = 0.0

        self.vs_electronics_mass = 2.4445*self.machine_rating + 1599.0

        self.hvac_mass = 0.08 * self.machine_rating

        self.controls_mass     = 0.0

        # mainframe system including bedplate, platforms, crane and miscellaneous hardware
        self.platforms_mass = 0.125 * self.bedplate_mass

        if (self.crane):
            self.crane_mass =  3000.0
        else:
            self.crane_mass = 0.0

        self.mainframe_mass  = self.bedplate_mass + self.crane_mass + self.platforms_mass

        nacelleCovArea      = 2 * (self.bedplate_length ** 2)              # this calculation is based on Sunderland
        self.cover_mass = (84.1 * nacelleCovArea) / 2          # this calculation is based on Sunderland - divided by 2 in order to approach CSM

        # yaw system weight calculations based on total system mass above yaw system
        self.above_yaw_mass =  self.lss_mass + \
                    self.main_bearing_mass + self.second_bearing_mass + \
                    self.gearbox_mass + \
                    self.hss_mass + \
                    self.generator_mass + \
                    self.mainframe_mass + \
                    self.electrical_mass + \
                    self.vs_electronics_mass + \
                    self.hvac_mass + \
                    self.cover_mass

        self.length      = self.bedplate_length                              # nacelle length [m] based on bedplate length
        self.width       = self.bedplate_width                        # nacelle width [m] based on bedplate width
        self.height      = (2.0 / 3.0) * self.length                         # nacelle height [m] calculated based on cladding area

        # derivatives
        self.d_hvac_mass_d_machine_rating = 0.08
        self.d_vs_mass_d_machine_rating = 2.4445
        self.d_platforms_mass_d_bedplate_mass = 0.125
        self.d_mainframe_mass_d_bedplate_mass = 1.0 + self.d_platforms_mass_d_bedplate_mass
        self.d_cover_mass_d_bedplate_length = (84.1 / 2) * (2*2) * self.bedplate_length
        self.d_above_yaw_mass_d_bedplate_mass = self.d_mainframe_mass_d_bedplate_mass
        self.d_above_yaw_mass_d_bedplate_length = self.d_cover_mass_d_bedplate_length
        self.d_above_yaw_mass_d_machine_rating = self.d_hvac_mass_d_machine_rating + self.d_vs_mass_d_machine_rating
        self.d_length_d_bedplate_length = 1.0
        self.d_width_d_bedplate_width = 1.0
        self.d_height_d_bedplate_length = (2.0/3.0)

    def list_deriv_vars(self):

        inputs = ['machine_rating','lss_mass', 'main_bearing_mass', 'second_bearing_mass', 'gearbox_mass', 'hss_mass', 'generator_mass', 'bedplate_mass', 'bedplate_length', 'bedplate_width']
        outputs = ['hvac_mass', 'vs_electronics_mass', 'platforms_mass', 'mainframe_mass', 'cover_mass', 'above_yaw_mass', 'length', 'width', 'height']

        return inputs, outputs

    def provideJ(self):

        # Jacobian
        self.J = np.array([[self.d_hvac_mass_d_machine_rating, 0, 0, 0, 0, 0, 0, 0, 0, 0], \
                          [self.d_vs_mass_d_machine_rating, 0, 0, 0, 0, 0, 0, 0, 0, 0], \
                          [0, 0, 0, 0, 0, 0, 0, self.d_platforms_mass_d_bedplate_mass, 0, 0], \
                          [0, 0, 0, 0, 0, 0, 0, self.d_mainframe_mass_d_bedplate_mass, 0, 0], \
                          [0, 0, 0, 0, 0, 0, 0, 0, self.d_cover_mass_d_bedplate_length, 0], \
                          [self.d_above_yaw_mass_d_machine_rating, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, self.d_above_yaw_mass_d_bedplate_mass, self.d_above_yaw_mass_d_bedplate_length, 0], \
                          [0, 0, 0, 0, 0, 0, 0, 0, self.d_length_d_bedplate_length, 0], \
                          [0, 0, 0, 0, 0, 0, 0, 0, 0, self.d_width_d_bedplate_width], \
                          [0, 0, 0, 0, 0, 0, 0, 0, self.d_height_d_bedplate_length, 0]])

        return self.J

class YawSystem(Component):
    ''' YawSystem class
          The YawSystem class is used to represent the yaw system of a wind turbine drivetrain.
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.
    '''

    # variables
    rotor_diameter = Float(iotype='in', units='m', desc='rotor diameter')
    rotor_thrust = Float(iotype='in', units='N', desc='maximum rotor thrust')
    tower_top_diameter = Float(iotype='in', units='m', desc='tower top diameter')
    above_yaw_mass = Float(iotype='in', units='kg', desc='above yaw mass')

    # returns
    mass = Float(0.0, iotype='out', units='kg', desc='overall component mass')
    cm = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system')
    I = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass')

    def __init__(self):
        '''
        Initializes yaw system component

        Parameters
        ----------
        rotor_diameter : float
          The wind turbine rotor diameter [m]
        rotor_thrust : float
          Maximum thrust from the rotor applied to the drivetrain under extreme conditions [N]
        tower_top_diameter : float
          Diameter of the turbine tower top [m]
        above_yaw_mass : float
          Mass of rotor and nacelle above yaw bearing [kg]

        Returns
        -------
        mass : float
          mass of the component [kg]
        cm : array of float
          center of mass of the component (relative to tower top center) [m, m, m]
        I : array of float
          principle moments of inertia for the component (Ixx, iyy, Izz) around its center of mass (relative to tower top center) [kg*m^2, kg*m^2, kg*m^2]
        '''

        super(YawSystem, self).__init__()

        #controls what happens if derivatives are missing
        self.missing_deriv_policy = 'assume_zero'

    def execute(self):

        # yaw weight depends on moment due to weight of components above yaw bearing and moment due to max thrust load
        #above_yaw_mass = 350000 # verboseging number based on 5 MW RNA mass
        yawfactor = 0.41 * (2.4 * (10 ** (-3)))                   # should depend on rotor configuration: blade number and hub type
        weightMom = self.above_yaw_mass * self.rotor_diameter                    # moment due to weight above yaw system
        thrustMom = self.rotor_thrust * self.tower_top_diameter                  # moment due to rotor thrust
        self.mass = (yawfactor * (0.4 * weightMom + 0.975 * thrustMom))

        # calculate mass properties
        # yaw system assumed to be collocated to tower top center
        cm = np.array([0.0,0.0,0.0])
        self.cm = cm

        I = np.array([0.0, 0.0, 0.0])
        self.I = I

        # derivatives
        self.d_mass_d_rotor_diameter = yawfactor * 0.4 * self.above_yaw_mass
        self.d_mass_d_rotor_thrust = yawfactor * 0.975 * self.tower_top_diameter
        self.d_mass_d_tower_top_diameter = yawfactor * 0.975 * self.rotor_thrust
        self.d_mass_d_above_yaw_mass = yawfactor * 0.4 * self.rotor_diameter

    def list_deriv_vars(self):

        inputs = ['rotor_diameter', 'rotor_thrust', 'tower_top_diameter', 'above_yaw_mass']
        outputs = ['mass']

        return inputs, outputs

    def provideJ(self):

        # Jacobian
        self.J = np.array([[self.d_mass_d_rotor_diameter, self.d_mass_d_rotor_thrust, self.d_mass_d_tower_top_diameter, self.d_mass_d_above_yaw_mass]])

        return self.J

#-------------------------------------------------------------------------------

class NacelleSystemAdder(Component): # changed name to nacelle - need to rename, move code pieces, develop configurations ***
    ''' NacelleSystem class
          The Nacelle class is used to represent the overall nacelle of a wind turbine.
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.
    '''
    # variables
    above_yaw_mass = Float(iotype='in', units='kg', desc='mass above yaw system')
    yawMass = Float(iotype='in', units='kg', desc='mass of yaw system')
    lss_mass = Float(iotype = 'in', units='kg', desc='component mass')
    main_bearing_mass = Float(iotype = 'in', units='kg', desc='component mass')
    second_bearing_mass = Float(iotype = 'in', units='kg', desc='component mass')
    gearbox_mass = Float(iotype = 'in', units='kg', desc='component mass')
    hss_mass = Float(iotype = 'in', units='kg', desc='component mass')
    generator_mass = Float(iotype = 'in', units='kg', desc='component mass')
    bedplate_mass = Float(iotype = 'in', units='kg', desc='component mass')
    mainframe_mass = Float(iotype = 'in', units='kg', desc='component mass')
    lss_cm = Array(np.array([0.0,0.0,0.0]),iotype = 'in', units='kg', desc='component CM')
    main_bearing_cm = Array(np.array([0.0,0.0,0.0]),iotype = 'in', units='kg', desc='component CM')
    second_bearing_cm = Array(np.array([0.0,0.0,0.0]),iotype = 'in', units='kg', desc='component CM')
    gearbox_cm = Array(np.array([0.0,0.0,0.0]),iotype = 'in', units='kg', desc='component CM')
    hss_cm = Array(np.array([0.0,0.0,0.0]),iotype = 'in', units='kg', desc='component CM')
    generator_cm = Array(np.array([0.0,0.0,0.0]),iotype = 'in', units='kg', desc='component CM')
    bedplate_cm = Array(np.array([0.0,0.0,0.0]),iotype = 'in', units='kg', desc='component CM')
    lss_I = Array(np.array([0.0,0.0,0.0]),iotype = 'in', units='kg', desc='component I')
    main_bearing_I = Array(np.array([0.0,0.0,0.0]),iotype = 'in', units='kg', desc='component I')
    second_bearing_I = Array(np.array([0.0,0.0,0.0]),iotype = 'in', units='kg', desc='component I')
    gearbox_I = Array(np.array([0.0,0.0,0.0]),iotype = 'in', units='kg', desc='component I')
    hss_I = Array(np.array([0.0,0.0,0.0]),iotype = 'in', units='kg', desc='component I')
    generator_I = Array(np.array([0.0,0.0,0.0]),iotype = 'in', units='kg', desc='component I')
    bedplate_I = Array(np.array([0.0,0.0,0.0]),iotype = 'in', units='kg', desc='component I')

    # returns
    mass = Float(0.0, iotype='out', units='kg', desc='overall component mass')
    cm = Array(np.array([0.0, 0.0, 0.0]), iotype='out', units='m', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system')
    I = Array(np.array([0.0, 0.0, 0.0]), iotype='out', units='kg*m**2', desc='moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass')

    def __init__(self):
        ''' Initialize above yaw mass adder component

        Parameters
        ----------
        above_yaw_mass : float
          total mass above yaw system [kg]
        yawMass : float
          mass of yaw component [kg]
        lss_mass : float
          low speed shaft component mass [kg]
        main_bearing_mass : float
          main bearing component mass [kg]
        second_bearing_mass : float
          second bearing component mass [kg]
        gearbox_mass : float
          gearbox component mass [kg]
        hss_mass : float
          high speed side component mass [kg]
        generator_mass : float
          generator component mass [kg]
        bedplate_mass : float
          bedplate component mass [kg]
        mainframe_mass : float
          mainframe component mass [kg]
        lss_cm : array of float
          low speed shaft component cm [m, m, m]
        main_bearing_cm : array of float
          main bearing component cm [m, m, m]
        second_bearing_cm : array of float
          second bearing component cm [m, m, m]
        gearbox_cm : array of float
          gearbox component cm [m, m, m]
        hss_cm : array of float
          high speed side component cm [m, m, m]
        generator_cm : array of float
          generator component cm [m, m, m]
        bedplate_cm : array of float
          bedplate component cm [m, m, m]
        lss_I : array of float
          low speed shaft component mass moments of inertia for principle axis
        main_bearing_I : array of float
          main bearing component mass moments of inertia for principle axis
        second_bearing_I : array of float
          second bearing component mass moments of inertia for principle axis
        gearbox_I : array of float
          gearbox component mass moments of inertia for principle axis
        hss_I : array of float
          high speed side component mass moments of inertia for principle axis
        generator_I : array of float
          generator component mass moments of inertia for principle axis
        bedplate_I : array of float
          bedplate component mass moments of inertia for principle axis

        Returns
        -------
        mass : float
          mass of the component [kg]
        cm : array of float
          center of mass of the component (relative to tower top center) [m, m, m]
        I : array of float
          principle moments of inertia for the component (Ixx, iyy, Izz) around its center of mass (relative to tower top center) [kg*m^2, kg*m^2, kg*m^2]
        '''

        super(NacelleSystemAdder , self).__init__()

        #controls what happens if derivatives are missing
        self.missing_deriv_policy = 'assume_zero'

    def execute(self):

        # aggregation of nacelle mass
        self.mass = (self.above_yaw_mass + self.yawMass)

        # calculation of mass center and moments of inertia
        cm = np.array([0.0,0.0,0.0])
        for i in (range(0,3)):
            # calculate center of mass (use mainframe_mass in place of bedplate_mass - assume lumped around bedplate_cm)
            cm[i] = (self.lss_mass * self.lss_cm[i] +
                    self.main_bearing_mass * self.main_bearing_cm[i] + self.second_bearing_mass * self.second_bearing_cm[i] + \
                    self.gearbox_mass * self.gearbox_cm[i] + self.hss_mass * self.hss_cm[i] + \
                    self.generator_mass * self.generator_cm[i] + self.mainframe_mass * self.bedplate_cm[i] ) / \
                    (self.lss_mass + self.main_bearing_mass + self.second_bearing_mass + \
                    self.gearbox_mass + self.hss_mass + self.generator_mass + self.mainframe_mass)
        self.cm = cm

        I = np.zeros(6)
        for i in (range(0,3)):                        # calculating MOI, at nacelle center of gravity with origin at tower top center / yaw mass center, ignoring masses of non-drivetrain components / auxiliary systems
            # calculate moments around CM
            # sum moments around each components CM (adjust for mass of mainframe) # TODO: add yaw MMI
            I[i]  =  self.lss_I[i] + self.main_bearing_I[i] + self.second_bearing_I[i] + self.gearbox_I[i] + \
                          self.hss_I[i] + self.generator_I[i] + self.bedplate_I[i] * (self.mainframe_mass / self.bedplate_mass)
            # translate to nacelle CM using parallel axis theorem (use mass of mainframe en lieu of bedplate to account for auxiliary equipment)
            for j in (range(0,3)):
                if i != j:
                    I[i] +=  self.lss_mass * (self.lss_cm[i] - cm[i]) ** 2 + \
                                  self.main_bearing_mass * (self.main_bearing_cm[i] - cm[i]) ** 2 + \
                                  self.second_bearing_mass * (self.second_bearing_cm[i] - cm[i]) ** 2 + \
                                  self.gearbox_mass * (self.gearbox_cm[i] - cm[i]) ** 2 + \
                                  self.hss_mass * (self.hss_cm[i] - cm[i]) ** 2 + \
                                  self.generator_mass * (self.generator_cm[i] - cm[i]) ** 2 + \
                                  self.mainframe_mass * (self.bedplate_cm[i] - cm[i]) ** 2
        self.I = I

        # derivatives
        self.d_mass_d_above_yaw_mass = 1.0
        self.d_mass_d_yawMass = 1.0

        sum_mass = self.lss_mass + self.main_bearing_mass + self.second_bearing_mass + self.gearbox_mass + self.hss_mass + self.generator_mass + self.mainframe_mass

        self.d_cm_d_lss_mass = np.array([0.0, 0.0, 0.0])
        for i in (range(0,3)):
            self.d_cm_d_lss_mass[i] = (self.lss_cm[i]*sum_mass - (self.lss_mass * self.lss_cm[i] + self.main_bearing_mass * self.main_bearing_cm[i] + \
                    self.second_bearing_mass * self.second_bearing_cm[i] + self.gearbox_mass * self.gearbox_cm[i] + self.hss_mass * self.hss_cm[i] + \
                    self.generator_mass * self.generator_cm[i] + self.mainframe_mass * self.bedplate_cm[i]))/ ((sum_mass)**2)
        self.d_cm_d_main_bearing_mass = np.array([0.0, 0.0, 0.0])
        for i in (range(0,3)):
            self.d_cm_d_main_bearing_mass[i] = (self.main_bearing_cm[i]*sum_mass - (self.lss_mass * self.lss_cm[i] + self.main_bearing_mass * self.main_bearing_cm[i] + \
                    self.second_bearing_mass * self.second_bearing_cm[i] + self.gearbox_mass * self.gearbox_cm[i] + self.hss_mass * self.hss_cm[i] + \
                    self.generator_mass * self.generator_cm[i] + self.mainframe_mass * self.bedplate_cm[i]))/ ((sum_mass)**2)
        self.d_cm_d_second_bearing_mass = np.array([0.0, 0.0, 0.0])
        for i in (range(0,3)):
            self.d_cm_d_second_bearing_mass[i] = (self.second_bearing_cm[i]*sum_mass - (self.lss_mass * self.lss_cm[i] + self.main_bearing_mass * self.main_bearing_cm[i] + \
                    self.second_bearing_mass * self.second_bearing_cm[i] + self.gearbox_mass * self.gearbox_cm[i] + self.hss_mass * self.hss_cm[i] + \
                    self.generator_mass * self.generator_cm[i] + self.mainframe_mass * self.bedplate_cm[i]))/ ((sum_mass)**2)
        self.d_cm_d_gearbox_mass = np.array([0.0, 0.0, 0.0])
        for i in (range(0,3)):
            self.d_cm_d_gearbox_mass[i] = (self.gearbox_cm[i]*sum_mass - (self.lss_mass * self.lss_cm[i] + self.main_bearing_mass * self.main_bearing_cm[i] + \
                    self.second_bearing_mass * self.second_bearing_cm[i] + self.gearbox_mass * self.gearbox_cm[i] + self.hss_mass * self.hss_cm[i] + \
                    self.generator_mass * self.generator_cm[i] + self.mainframe_mass * self.bedplate_cm[i]))/ ((sum_mass)**2)
        self.d_cm_d_hss_mass = np.array([0.0, 0.0, 0.0])
        for i in (range(0,3)):
            self.d_cm_d_hss_mass[i] = (self.hss_cm[i]*sum_mass - (self.lss_mass * self.lss_cm[i] + self.main_bearing_mass * self.main_bearing_cm[i] + \
                    self.second_bearing_mass * self.second_bearing_cm[i] + self.gearbox_mass * self.gearbox_cm[i] + self.hss_mass * self.hss_cm[i] + \
                    self.generator_mass * self.generator_cm[i] + self.mainframe_mass * self.bedplate_cm[i]))/ ((sum_mass)**2)
        self.d_cm_d_generator_mass = np.array([0.0, 0.0, 0.0])
        for i in (range(0,3)):
            self.d_cm_d_generator_mass[i] = (self.generator_cm[i]*sum_mass - (self.lss_mass * self.lss_cm[i] + self.main_bearing_mass * self.main_bearing_cm[i] + \
                    self.second_bearing_mass * self.second_bearing_cm[i] + self.gearbox_mass * self.gearbox_cm[i] + self.hss_mass * self.hss_cm[i] + \
                    self.generator_mass * self.generator_cm[i] + self.mainframe_mass * self.bedplate_cm[i]))/ ((sum_mass)**2)
        self.d_cm_d_mainframe_mass = np.array([0.0, 0.0, 0.0])
        for i in (range(0,3)):
            self.d_cm_d_mainframe_mass[i] = (self.bedplate_cm[i]*sum_mass - (self.lss_mass * self.lss_cm[i] + self.main_bearing_mass * self.main_bearing_cm[i] + \
                    self.second_bearing_mass * self.second_bearing_cm[i] + self.gearbox_mass * self.gearbox_cm[i] + self.hss_mass * self.hss_cm[i] + \
                    self.generator_mass * self.generator_cm[i] + self.mainframe_mass * self.bedplate_cm[i]))/ ((sum_mass)**2)

        self.d_cm_d_lss_cm = self.lss_mass / sum_mass
        self.d_cm_d_main_bearing_cm = self.main_bearing_mass / sum_mass
        self.d_cm_d_second_bearing_cm = self.second_bearing_mass / sum_mass
        self.d_cm_d_gearbox_cm = self.gearbox_mass / sum_mass
        self.d_cm_d_hss_cm = self.hss_mass / sum_mass
        self.d_cm_d_generator_cm = self.generator_mass / sum_mass
        self.d_cm_d_mainframeCM = self.mainframe_mass / sum_mass

        self.d_I_d_lss_I = 1
        self.d_I_d_main_bearing_I = 1
        self.d_I_d_second_bearing_I = 1
        self.d_I_d_gearbox_I = 1
        self.d_I_d_hss_I = 1
        self.d_I_d_generator_I = 1
        self.d_I_d_mainframeI = 1

        self.d_I_d_lss_mass = np.array([0.0, 0.0, 0.0])
        for i in (range(0,3)):
            self.d_I_d_lss_mass[i] = (self.lss_cm[i]-self.cm[i])**2 + 2 * self.lss_mass * (self.lss_cm[i] - self.cm[i]) * self.d_cm_d_lss_cm
        self.d_I_d_main_bearing_mass = np.array([0.0, 0.0, 0.0])
        for i in (range(0,3)):
            self.d_I_d_main_bearing_mass[i] = (self.main_bearing_cm[i]-self.cm[i])**2 + 2 * self.main_bearing_mass * (self.main_bearing_cm[i] - self.cm[i]) * self.d_cm_d_main_bearing_cm
        self.d_I_d_second_bearing_mass = np.array([0.0, 0.0, 0.0])
        for i in (range(0,3)):
            self.d_I_d_second_bearing_mass[i] = (self.second_bearing_cm[i]-self.cm[i])**2 + 2 * self.second_bearing_mass * (self.second_bearing_cm[i] - self.cm[i]) * self.d_cm_d_second_bearing_cm
        self.d_I_d_gearbox_mass = np.array([0.0, 0.0, 0.0])
        for i in (range(0,3)):
            self.d_I_d_gearbox_mass[i] = (self.gearbox_cm[i]-self.cm[i])**2 + 2 * self.gearbox_mass * (self.gearbox_cm[i] - self.cm[i]) * self.d_cm_d_gearbox_cm
        self.d_I_d_hss_mass = np.array([0.0, 0.0, 0.0])
        for i in (range(0,3)):
            self.d_I_d_hss_mass[i] = (self.hss_cm[i]-self.cm[i])**2 + 2 * self.hss_mass * (self.hss_cm[i] - self.cm[i]) * self.d_cm_d_hss_cm
        self.d_I_d_generator_mass = np.array([0.0, 0.0, 0.0])
        for i in (range(0,3)):
            self.d_I_d_generator_mass[i] = (self.generator_cm[i]-self.cm[i])**2 + 2 * self.generator_mass * (self.generator_cm[i] - self.cm[i]) * self.d_cm_d_generator_cm
        self.d_I_d_mainframe_mass = np.array([0.0, 0.0, 0.0])
        for i in (range(0,3)):
            self.d_I_d_mainframe_mass[i] = (self.bedplate_cm[i]-self.cm[i])**2 + 2 * self.mainframe_mass * (self.bedplate_cm[i] - self.cm[i]) * self.d_cm_d_mainframeCM + \
                                           self.bedplate_I[i] / self.bedplate_mass


        self.d_I_d_lss_cm = np.array([0.0, 0.0, 0.0])
        for i in range(0,3):
            self.d_I_d_lss_cm[i] = 2 * self.lss_mass * (self.lss_cm[i] - self.cm[i]) * (1 - self.d_cm_d_lss_cm)
        self.d_I_d_main_bearing_cm = np.array([0.0, 0.0, 0.0])
        for i in range(0,3):
            self.d_I_d_main_bearing_cm[i] = 2 * self.main_bearing_mass * (self.main_bearing_cm[i] - self.cm[i]) * (1 - self.d_cm_d_main_bearing_cm)
        self.d_I_d_second_bearing_cm = np.array([0.0, 0.0, 0.0])
        for i in range(0,3):
            self.d_I_d_second_bearing_cm[i] = 2 * self.second_bearing_mass * (self.second_bearing_cm[i] - self.cm[i]) * (1 - self.d_cm_d_second_bearing_cm)
        self.d_I_d_gearbox_cm = np.array([0.0, 0.0, 0.0])
        for i in range(0,3):
            self.d_I_d_gearbox_cm[i] = 2 * self.gearbox_mass * (self.gearbox_cm[i] - self.cm[i]) * (1 - self.d_cm_d_gearbox_cm)
        self.d_I_d_hss_cm = np.array([0.0, 0.0, 0.0])
        for i in range(0,3):
            self.d_I_d_hss_cm[i] = 2 * self.hss_mass * (self.hss_cm[i] - self.cm[i]) * (1 - self.d_cm_d_hss_cm)
        self.d_I_d_generator_cm = np.array([0.0, 0.0, 0.0])
        for i in range(0,3):
            self.d_I_d_generator_cm[i] = 2 * self.generator_mass * (self.generator_cm[i] - self.cm[i]) * (1 - self.d_cm_d_generator_cm)
        self.d_I_d_mainframeCM = np.array([0.0, 0.0, 0.0])
        for i in range(0,3):
            self.d_I_d_mainframeCM[i] = 2 * self.mainframe_mass * (self.bedplate_cm[i] - self.cm[i]) * (1 - self.d_cm_d_mainframeCM)

    def list_deriv_vars(self):

        inputs = ['above_yaw_mass', 'yawMass', 'lss_mass', 'main_bearing_mass', 'second_bearing_mass', 'gearbox_mass', 'hss_mass', 'generator_mass', 'mainframe_mass', \
                      'lss_cm', 'main_bearing_cm', 'second_bearing_cm', 'gearbox_cm', 'hss_cm', 'generator_cm', 'bedplate_cm', \
                      'lss_I', 'main_bearing_I', 'second_bearing_I', 'gearbox_I', 'hss_I', 'generator_I', 'bedplate_I']
        outputs = ['mass', 'cm', 'I']

        return inputs, outputs

    def provideJ(self):

        # Jacobian
        self.J = np.array([[self.d_mass_d_above_yaw_mass, self.d_mass_d_yawMass, 0, 0, 0, 0, 0, 0, 0, \
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, \
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], \
                           [0, 0, self.d_cm_d_lss_mass[0], self.d_cm_d_main_bearing_mass[0], self.d_cm_d_second_bearing_mass[0], self.d_cm_d_gearbox_mass[0], self.d_cm_d_hss_mass[0], self.d_cm_d_generator_mass[0], self.d_cm_d_mainframe_mass[0], \
                            self.d_cm_d_lss_cm, 0, 0, self.d_cm_d_main_bearing_cm, 0, 0, self.d_cm_d_second_bearing_cm, 0, 0, self.d_cm_d_gearbox_cm, 0, 0, self.d_cm_d_hss_cm, 0, 0, self.d_cm_d_generator_cm, 0, 0, self.d_cm_d_mainframeCM, 0, 0, \
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], \
                           [0, 0, self.d_cm_d_lss_mass[1], self.d_cm_d_main_bearing_mass[1], self.d_cm_d_second_bearing_mass[1], self.d_cm_d_gearbox_mass[1], self.d_cm_d_hss_mass[1], self.d_cm_d_generator_mass[1], self.d_cm_d_mainframe_mass[1], \
                            0, self.d_cm_d_lss_cm, 0, 0, self.d_cm_d_main_bearing_cm, 0, 0, self.d_cm_d_second_bearing_cm, 0, 0, self.d_cm_d_gearbox_cm, 0, 0, self.d_cm_d_hss_cm, 0, 0, self.d_cm_d_generator_cm, 0, 0, self.d_cm_d_mainframeCM, 0, \
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], \
                           [0, 0, self.d_cm_d_lss_mass[2], self.d_cm_d_main_bearing_mass[2], self.d_cm_d_second_bearing_mass[2], self.d_cm_d_gearbox_mass[2], self.d_cm_d_hss_mass[2], self.d_cm_d_generator_mass[2], self.d_cm_d_mainframe_mass[2], \
                            0, 0, self.d_cm_d_lss_cm, 0, 0, self.d_cm_d_main_bearing_cm, 0, 0, self.d_cm_d_second_bearing_cm, 0, 0, self.d_cm_d_gearbox_cm, 0, 0, self.d_cm_d_hss_cm, 0, 0, self.d_cm_d_generator_cm, 0, 0, self.d_cm_d_mainframeCM, \
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], \
                           [0, 0, self.d_I_d_lss_mass[0], self.d_I_d_main_bearing_mass[0], self.d_I_d_second_bearing_mass[0], self.d_I_d_gearbox_mass[0], self.d_I_d_hss_mass[0], self.d_I_d_generator_mass[0], self.d_I_d_mainframe_mass[0], \
                           self.d_I_d_lss_cm[0], 0, 0, self.d_I_d_main_bearing_cm[0], 0, 0, self.d_I_d_second_bearing_cm[0], 0, 0, self.d_I_d_gearbox_cm[0], 0, 0, self.d_I_d_hss_cm[0], 0, 0, self.d_I_d_generator_cm[0], 0, 0, self.d_I_d_mainframeCM[0], 0, 0, \
                           self.d_I_d_lss_I, 0, 0, self.d_I_d_main_bearing_I, 0, 0, self.d_I_d_second_bearing_I, 0, 0, self.d_I_d_gearbox_I, 0, 0, self.d_I_d_hss_I, 0, 0, self.d_I_d_generator_I, 0, 0, self.d_I_d_mainframeI, 0, 0], \
                           [0, 0, self.d_I_d_lss_mass[1], self.d_I_d_main_bearing_mass[1], self.d_I_d_second_bearing_mass[1], self.d_I_d_gearbox_mass[1], self.d_I_d_hss_mass[1], self.d_I_d_generator_mass[1], self.d_I_d_mainframe_mass[1], \
                           0, self.d_I_d_lss_cm[1], 0, 0, self.d_I_d_main_bearing_cm[1], 0, 0, self.d_I_d_second_bearing_cm[1], 0, 0, self.d_I_d_gearbox_cm[1], 0, 0, self.d_I_d_hss_cm[1], 0, 0, self.d_I_d_generator_cm[1], 0, 0, self.d_I_d_mainframeCM[1], 0, \
                           0, self.d_I_d_lss_I, 0, 0, self.d_I_d_main_bearing_I, 0, 0, self.d_I_d_second_bearing_I, 0, 0, self.d_I_d_gearbox_I, 0, 0, self.d_I_d_hss_I, 0, 0, self.d_I_d_generator_I, 0, 0, self.d_I_d_mainframeI, 0], \
                           [0, 0, self.d_I_d_lss_mass[2], self.d_I_d_main_bearing_mass[2], self.d_I_d_second_bearing_mass[2], self.d_I_d_gearbox_mass[2], self.d_I_d_hss_mass[2], self.d_I_d_generator_mass[2], self.d_I_d_mainframe_mass[2], \
                           0, 0, self.d_I_d_lss_cm[2], 0, 0, self.d_I_d_main_bearing_cm[2], 0, 0, self.d_I_d_second_bearing_cm[2], 0, 0, self.d_I_d_gearbox_cm[2], 0, 0, self.d_I_d_hss_cm[2], 0, 0, self.d_I_d_generator_cm[2], 0, 0, self.d_I_d_mainframeCM[2], \
                           0, 0, self.d_I_d_lss_I, 0, 0, self.d_I_d_main_bearing_I, 0, 0, self.d_I_d_second_bearing_I, 0, 0, self.d_I_d_gearbox_I, 0, 0, self.d_I_d_hss_I, 0, 0, self.d_I_d_generator_I, 0, 0, self.d_I_d_mainframeI]])

        return self.J

#--------------------------------------------------------------------------------------------------------

if __name__ == '__main__':
     pass