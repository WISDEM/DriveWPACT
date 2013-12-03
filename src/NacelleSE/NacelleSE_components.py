"""
nacelleSE_components.py

Created by Katherine Dykes 2012.
Copyright (c) NREL. All rights reserved.
"""

from openmdao.main.api import Component, Assembly
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
    rotorDiameter = Float(iotype='in', units='m', desc='rotor diameter')
    rotorMass = Float(iotype='in', units='kg', desc='rotor mass')
    rotorTorque = Float(iotype='in', units='N*m', desc='rotor torque at rated power')
    
    # returns
    designTorque = Float(iotype='out', units='N*m', desc='lss design torque')
    designBendingLoad = Float(iotype='out', units='N', desc='lss design bending load')
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
        rotorDiameter : float
          The wind turbine rotor diameter [m]
        rotorMass : float
          The wind turbine rotor mass [kg]
        rotorTorque : float
          The input torque from the wind turbine at rated power after accounting for drivetrain losses [N*m]

        Returns
        -------
        designTorque : float
          Design torque for the low speed shaft based on input torque from the rotor at rated speed accounting for drivetrain losses - multiplied by a safety factor
        designBendingLoad : float
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
        
        super(LowSpeedShaft,self).__init__()
    
    def execute(self):

        # compute masses, dimensions and cost
        ioratio   = 0.100                                    # constant value for inner/outer diameter ratio (should depend on LSS type)                                                  
        hollow    = 1/(1-(ioratio)**4)                    # hollowness factor based on diameter ratio

        TQsafety       = 3.0                                    # safety factor for design torque applied to rotor torque
        self.designTorque =(TQsafety * self.rotorTorque)            # LSS design torque [Nm]

        lenFact        = 0.03                                   # constant value for length as a function of rotor diameter (should depend on LSS type) 
        self.length = (lenFact * self.rotorDiameter )              # LSS shaft length [m]                                                                  
        maFact         = 5                                 # moment arm factor from shaft lenght (should depend on shaft type)
        mmtArm    = self.length / maFact            # LSS moment arm [m] - from hub to first main bearing
        BLsafety       = 1.25                                   # saftey factor on bending load
        g              = 9.81                              # gravitational constant [m / s^2]
        self.designBendingLoad  = BLsafety * g * self.rotorMass          # LSS design bending load [N]                                                  
        bendMom   = self.designBendingLoad * mmtArm       # LSS design bending moment [Nm]

        yieldst        = 371000000.0                             # BS1503-622 yield stress [Pa] (should be adjusted depending on material type)
        endurstsp      = 309000000.0                             # BS1503-625 specimen endurance limit [Pa] (should be adjusted depending on material type)
        endurFact      = 0.23                                    # factor for specimen to component endurance limit 
                                                                # (0.75 surface condition * 0.65 size * 0.52 reliability * 1 temperature * 0.91 stress concentration)
        endurst        = endurstsp * endurFact                   # endurance limit [Pa] for LSS
        SOsafety       = 3.25                               # Soderberg Line approach factor for safety 
        self.diameter = ((32/pi)*hollow*SOsafety*((self.designTorque / yieldst)**2+(bendMom/endurst)**2)**(0.5))**(1./3.)                               
                                                            # outer diameter [m] computed by Westinghouse Code Formula based on Soderberg Line approach to fatigue design
        inDiam    = self.diameter * ioratio            # inner diameter [m]

        
        massFact       = 1.25                                    # mass weight factor (should depend on LSS/drivetrain type, currently from windpact modifications to account for flange weight)                                                                       
        steeldens      = 7860                                    # steel density [kg / m^3]

        self.mass = (massFact*(pi/4)*(self.diameter**2-inDiam**2)*self.length*steeldens)      # mass of LSS [kg]  

        # calculate mass properties        
        cm = np.array([0.0,0.0,0.0])
        cm[0] = - (0.035 - 0.01) * self.rotorDiameter            # cm based on WindPACT work - halfway between locations of two main bearings
        cm[1] = 0.0
        cm[2] = 0.025 * self.rotorDiameter                      
        self.cm = cm

        I = np.array([0.0, 0.0, 0.0])
        I[0]  = self.mass * (inDiam ** 2 + self.diameter ** 2) / 8
        I[1]  = self.mass * (inDiam ** 2 + self.diameter ** 2 + (4 / 3) * (self.length ** 2)) / 16
        I[2]  = I[1]
        self.I = I

        # derivatives
        d_length_d_rotorDiameter = lenFact
        d_torque_d_rotorTorque = TQsafety
        d_bending_d_rotorMass = BLsafety * g

        d_diameter_d_rotorDiameter = ((1/6) * ((32/pi) * hollow*SOsafety)**(1/3)) * (((self.designTorque/yieldst)**2 + (bendMom/endurst)**2)**(-5/6)) * (2 * bendMom / (endurst**2)) * (self.designBendingLoad * d_length_d_rotorDiameter / maFact)
        d_diameter_d_rotorTorque = ((1/6) * ((32/pi) * hollow * SOsafety)**(1/3)) * (((self.designTorque/yieldst)**2 + (bendMom/endurst)**2)**(-5/6)) * (2 * self.designTorque / (yieldst**2)) * (d_torque_d_rotorTorque)
        d_diameter_d_rotorMass = ((1/6) * ((32/pi) * hollow * SOsafety)**(1/3)) * (((self.designTorque/yieldst)**2 + (bendMom/endurst)**2)**(-5/6)) * (2 * bendMom / (endurst**2)) * (mmtArm * d_bending_d_rotorMass)

        d_lssMass_d_rotorDiameter = massFact * (pi/4) * (1-ioratio**2) * steeldens * (self.diameter**2) * d_length_d_rotorDiameter + \
                                    2 * massFact * (pi/4) * (1-ioratio**2) * steeldens * self.length * self.diameter * d_diameter_d_rotorDiameter
        d_lssMass_d_rotorTorque = 2 * massFact * (pi/4) * (1-ioratio**2) * steeldens * self.length * self.diameter * d_diameter_d_rotorTorque    
        d_lssMass_d_rotorMass =  2 * massFact * (pi/4) * (1-ioratio**2) * steeldens * self.length * self.diameter * d_diameter_d_rotorMass
                                  
        d_cm_d_rotorDiameter = np.array([-(0.035 - 0.01), 0.0, 0.025])

        d_I_d_rotorDiameter = np.array([0.0, 0.0, 0.0])
        d_I_d_rotorDiameter[0] = (1/8) * (1+ ioratio**2) * d_lssMass_d_rotorDiameter * (self.diameter**2) + (1/8) * (1+ioratio**2) * self.mass * 2 * self.diameter * d_diameter_d_rotorDiameter
        d_I_d_rotorDiameter[1] = (1/2) * d_I_d_rotorDiameter[0] + (1/16) * (4/3) * 2 * self.length * d_length_d_rotorDiameter
        d_I_d_rotorDiameter[2] = d_I_d_rotorDiameter[1]

        d_I_d_rotorTorque = np.array([0.0, 0.0, 0.0])
        d_I_d_rotorTorque[0] = (1/8) * (1+ ioratio**2) * d_lssMass_d_rotorTorque * (self.diameter**2) + (1/8) * (1+ioratio**2) * self.mass * 2 * self.diameter * d_diameter_d_rotorTorque
        d_I_d_rotorTorque[1] = (1/2) * d_I_d_rotorTorque[0]
        d_I_d_rotorTorque[2] = d_I_d_rotorTorque[1]

        d_I_d_rotorMass = np.array([0.0, 0.0, 0.0])
        d_I_d_rotorMass[0] = (1/8) * (1+ ioratio**2) * d_lssMass_d_rotorMass * (self.diameter**2) + (1/8) * (1+ioratio**2) * self.mass * 2 * self.diameter * d_diameter_d_rotorMass
        d_I_d_rotorMass[1] = (1/2) * d_I_d_rotorMass[0]
        d_I_d_rotorMass[2] = d_I_d_rotorMass[1]
        
        # Jacobian
        self.J = np.array([[d_length_d_rotorDiameter, 0, 0], \
                           [0, d_torque_d_rotorTorque, 0], \
                           [0, 0, d_bending_d_rotorMass], \
                           [d_lssMass_d_rotorDiameter, d_lssMass_d_rotorTorque, d_lssMass_d_rotorMass], \
                           [d_cm_d_rotorDiameter[0], 0, 0], \
                           [0, d_cm_d_rotorDiameter[1], 0], \
                           [0, 0, d_cm_d_rotorDiameter[2]], \
                           [d_I_d_rotorDiameter[0], d_I_d_rotorTorque[0], d_I_d_rotorMass[0]], \
                           [d_I_d_rotorDiameter[1], d_I_d_rotorTorque[1], d_I_d_rotorMass[1]], \
                           [d_I_d_rotorDiameter[2], d_I_d_rotorTorque[2], d_I_d_rotorMass[2]]])

    def provideJ(self):

        input_keys = ['rotorDiameter', 'rotorTorque', 'rotorMass']
        output_keys = ['length', 'designTorque', 'designBendingLoad', 'mass', 'cm[0]', 'cm[1]', 'cm[2]', 'I[0]', 'I[1]', 'I[2]']

        self.derivatives.set_first_derivative(input_keys, output_keys, self.J)

#-------------------------------------------------------------------------------

class MainBearing(Component): 
    ''' MainBearings class          
          The MainBearings class is used to represent the main bearing components of a wind turbine drivetrain. It contains two subcomponents (main bearing and second bearing) which also inherit from the SubComponent class.
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.           
    '''
    # variables
    lssDesignTorque = Float(iotype='in', units='N*m', desc='lss design torque')
    lssDiameter = Float(iotype='in', units='m', desc='lss outer diameter')
    lowSpeedShaftMass = Float(iotype='in', units='kg', desc='lss mass')
    rotorSpeed = Float(iotype='in', units='m/s', desc='rotor speed at rated')
    rotorDiameter = Float(iotype='in', units='m', desc='rotor diameter')
    
    # returns
    mass = Float(0.0, iotype='out', units='kg', desc='overall component mass')
    cm = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system')
    I = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass')
    
    def __init__(self):
        ''' Initializes main bearing component 

        Parameters
        ----------
        lssDesignTorque : float
          Low speed shaft design torque [N*m]
        lssDiameter : float
          Low speed shaft diameter [m]
        lowSpeedShaftMass : float
          Low speed shaft mass [kg]
        rotorSpeed : float
          Speed of the rotor at rated power [rpm]
        rotorDiameter : float
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
    
    def execute(self):
      
        # compute masses, dimensions and cost
        g = 9.81                                           # gravitational constant [m / s^2]
        design1SL = (4.0 / 3.0) * self.lssDesignTorque + self.lowSpeedShaftMass * (g / 2.0)
                                                           # front bearing static design load [N] based on default equation (should depend on LSS type)
        design1DL = 2.29 * design1SL * (self.rotorSpeed ** 0.3)
                                                           # front bearing dynamic design load [N]

        ratingDL  = 17.96 * ((self.lssDiameter * 1000.0) ** 1.9752)  # basic dynamic load rating for a bearing given inside diameter based on catalogue regression

        massFact  = 0.25                                 # bearing weight factor (should depend on drivetrain type) - using to adjust data closer to cost and scaling model estimates

        if (design1DL < ratingDL):
            b1mass = massFact * (26.13 * (10 ** (-6))) * ((self.lssDiameter * 1000.0) ** 2.77)
                                                           # bearing mass [kg] for single row bearing (based on catalogue data regression)
            h1mass = massFact * (67.44 * (10 ** (-6))) * ((self.lssDiameter * 1000.0) ** 2.64)
                                                           # bearing housing mass [kg] for single row bearing (based on catalogue data regression) 
        else:
            b1mass = massFact * 1.7 * (26.13 * (10 ** (-6))) * ((self.lssDiameter * 1000.0) ** 2.77)
                                                          # bearing mass [kg] for double row bearing (based on catalogue data regression) 
            h1mass = massFact * 1.5 * (67.44 * (10 ** (-6))) * ((self.lssDiameter * 1000.0) ** 2.64)
                                                          # bearing housing mass [kg] for double row bearing (based on catalogue data regression) 

        self.mass = b1mass + h1mass

        # calculate mass properties
        inDiam  = self.lssDiameter
        depth = (inDiam * 1.5)

        cmMB = np.array([0.0,0.0,0.0])
        cmMB = ([- (0.035 * self.rotorDiameter), 0.0, 0.025 * self.rotorDiameter])
        self.cm = cmMB
       
        b1I0 = (b1mass * inDiam ** 2 ) / 4 + (h1mass * depth ** 2) / 4
        self.I = ([b1I0, b1I0 / 2, b1I0 / 2])

        # derivatives
        if design1DL < ratingDL :
            d_mass_d_lssDiameter = 2.77 * 1000.0 * (massFact * (26.13 * (10 ** (-6))) * ((self.lssDiameter * 1000.0) ** 1.77)) + \
                                   2.64 * 1000.0 * (massFact * (67.44 * (10 ** (-6))) * ((self.lssDiameter * 1000.0) ** 1.64))
        else :
            d_mass_d_lssDiameter = 2.77 * 1000.0 * (1.7 * massFact * (26.13 * (10 ** (-6))) * ((self.lssDiameter * 1000.0) ** 1.77)) + \
                                   2.64 * 1000.0 * (1.5 * massFact * (67.44 * (10 ** (-6))) * ((self.lssDiameter * 1000.0) ** 1.64))
        
        d_cm_d_rotorDiameter = np.array([-0.035, 0.0, 0.025])
        d_I_d_lssDiameter = np.array([0.0, 0.0, 0.0])
        if design1DL < ratingDL :
            d_I_d_lssDiameter[0] = 2.77 * 1000.0 * (massFact * (26.13 * (10 ** (-6))) * ((self.lssDiameter * 1000.0) ** 1.77)) * ((inDiam**2)/4) + self.mass * (2/4) * inDiam + \
                                   2.64 * 1000.0 * (massFact * (67.44 * (10 ** (-6))) * ((self.lssDiameter * 1000.0) ** 1.64)) * ((depth**2)/4) + self.mass * (2/4) * 1.5 * depth
        else :
            d_I_d_lssDiameter[0] = 2.77 * 1000.0 * (1.7 * massFact * (26.13 * (10 ** (-6))) * ((self.lssDiameter * 1000.0) ** 1.77)) * ((inDiam**2)/4) + self.mass * (2/4) * inDiam + \
                                      2.64 * 1000.0 * (1.5 * massFact * (67.44 * (10 ** (-6))) * ((self.lssDiameter * 1000.0) ** 1.64)) * ((depth**2)/4) + self.mass * (2/4) * 1.5 * depth
        d_I_d_lssDiameter[1] = (1/2) * d_I_d_lssDiameter[0]
        d_I_d_lssDiameter[2] = (1/2) * d_I_d_lssDiameter[0]

                               
        
        # Jacobian
        self.J = np.array([[d_mass_d_lssDiameter, 0], \
                           [0, d_cm_d_rotorDiameter[0]], \
                           [0, d_cm_d_rotorDiameter[1]], \
                           [0, d_cm_d_rotorDiameter[2]], \
                           [d_I_d_lssDiameter[0], 0], \
                           [d_I_d_lssDiameter[1], 0], \
                           [d_I_d_lssDiameter[2], 0]])

    def provideJ(self):

        input_keys = ['lssDiameter', 'rotorDiameter']
        output_keys = ['mass', 'cm[0]', 'cm[1]', 'cm[2]', 'I[0]', 'I[1]', 'I[2]']

        self.derivatives.set_first_derivative(input_keys, output_keys, self.J)

#-------------------------------------------------------------------------------

class SecondBearing(Component): 
    ''' MainBearings class          
          The MainBearings class is used to represent the main bearing components of a wind turbine drivetrain. It contains two subcomponents (main bearing and second bearing) which also inherit from the SubComponent class.
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.           
    '''
    # variables
    lssDesignTorque = Float(iotype='in', units='N*m', desc='lss design torque')
    lssDiameter = Float(iotype='in', units='m', desc='lss outer diameter')
    lowSpeedShaftMass = Float(iotype='in', units='kg', desc='lss mass')
    rotorSpeed = Float(iotype='in', units='m/s', desc='rotor speed at rated')
    rotorDiameter = Float(iotype='in', units='m', desc='rotor diameter')
    
    # returns
    mass = Float(0.0, iotype='out', units='kg', desc='overall component mass')
    cm = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system')
    I = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass')
    
    def __init__(self):
        ''' Initializes second bearing component 

        Parameters
        ----------
        lssDesignTorque : float
          Low speed shaft design torque [N*m]
        lssDiameter : float
          Low speed shaft diameter [m]
        lowSpeedShaftMass : float
          Low speed shaft mass [kg]
        rotorSpeed : float
          Speed of the rotor at rated power [rpm]
        rotorDiameter : float
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
    
    def execute(self):
      
        # compute masses, dimensions and cost
        g = 9.81                                           # gravitational constant [m / s^2]
        design2SL = (1.0 / 3.0) * self.lssDesignTorque - self.lowSpeedShaftMass * (g / 2.0)
                                                           # rear bearing static design load [N] based on default equation (should depend on LSS type)
        design2DL = 2.29 * design2SL * (self.rotorSpeed ** 0.3)
                                                           # rear bearing dynamic design load [N]
        ratingDL  = 17.96 * ((self.lssDiameter * 1000.0) ** 1.9752)  # basic dynamic load rating for a bearing given inside diameter based on catalogue regression
        massFact  = 0.25                                 # bearing weight factor (should depend on drivetrain type) - using to adjust data closer to cost and scaling model estimates 

        if (design2DL < ratingDL):
            b2mass = massFact * (26.13 * (10 ** (-6))) * ((self.lssDiameter * 1000.0) ** 2.77)
                                                           # bearing mass [kg] for single row bearing (based on catalogue data regression)
            h2mass = massFact * (67.44 * (10 ** (-6))) * ((self.lssDiameter * 1000.0) ** 2.64)
                                                           # bearing housing mass [kg] for single row bearing (based on catalogue data regression) 
        else:
            b2mass = massFact * 1.7 * (26.13 * (10 ** (-6))) * ((self.lssDiameter * 1000.0) ** 2.77)
                                                          # bearing mass [kg] for double row bearing (based on catalogue data regression) 
            h2mass = massFact * 1.5 * (67.44 * (10 ** (-6))) * ((self.lssDiameter * 1000.0) ** 2.64)
                                                          # bearing housing mass [kg] for double row bearing (based on catalogue data regression) 

        self.mass = b2mass + h2mass

        # calculate mass properties
        inDiam  = self.lssDiameter
        depth = (inDiam * 1.5)

        cmSB = np.array([0.0,0.0,0.0])
        cmSB = ([- (0.01 * self.rotorDiameter), 0.0, 0.025 * self.rotorDiameter])
        self.cm = cmSB

        b2I0  = (b2mass * inDiam ** 2 ) / 4 + (h2mass * depth ** 2) / 4
        self.I = ([b2I0, b2I0 / 2, b2I0 / 2])

        # derivatives
        if design2DL < ratingDL :
            d_mass_d_lssDiameter = 2.77 * 1000.0 * (massFact * (26.13 * (10 ** (-6))) * ((self.lssDiameter * 1000.0) ** 1.77)) + \
                                   2.64 * 1000.0 * (massFact * (67.44 * (10 ** (-6))) * ((self.lssDiameter * 1000.0) ** 1.64))
        else :
            d_mass_d_lssDiameter = 2.77 * 1000.0 * (1.7 * massFact * (26.13 * (10 ** (-6))) * ((self.lssDiameter * 1000.0) ** 1.77)) + \
                                   2.64 * 1000.0 * (1.5 * massFact * (67.44 * (10 ** (-6))) * ((self.lssDiameter * 1000.0) ** 1.64))
        
        d_cm_d_rotorDiameter = np.array([-0.01, 0.0, 0.025])
        d_I_d_lssDiameter = np.array([0.0, 0.0, 0.0])
        if design2DL < ratingDL :
            d_I_d_lssDiameter[0] = 2.77 * 1000.0 * (massFact * (26.13 * (10 ** (-6))) * ((self.lssDiameter * 1000.0) ** 1.77)) * ((inDiam**2)/4) + self.mass * (2/4) * inDiam + \
                                   2.64 * 1000.0 * (massFact * (67.44 * (10 ** (-6))) * ((self.lssDiameter * 1000.0) ** 1.64)) * ((depth**2)/4) + self.mass * (2/4) * 1.5 * depth
        else :
            d_I_d_lssDiameter[0] = 2.77 * 1000.0 * (1.7 * massFact * (26.13 * (10 ** (-6))) * ((self.lssDiameter * 1000.0) ** 1.77)) * ((inDiam**2)/4) + self.mass * (2/4) * inDiam + \
                                      2.64 * 1000.0 * (1.5 * massFact * (67.44 * (10 ** (-6))) * ((self.lssDiameter * 1000.0) ** 1.64)) * ((depth**2)/4) + self.mass * (2/4) * 1.5 * depth
        d_I_d_lssDiameter[1] = (1/2) * d_I_d_lssDiameter[0]
        d_I_d_lssDiameter[2] = (1/2) * d_I_d_lssDiameter[0]                             
        
        # Jacobian
        self.J = np.array([[d_mass_d_lssDiameter, 0], \
                           [0, d_cm_d_rotorDiameter[0]], \
                           [0, d_cm_d_rotorDiameter[1]], \
                           [0, d_cm_d_rotorDiameter[2]], \
                           [d_I_d_lssDiameter[0], 0], \
                           [d_I_d_lssDiameter[1], 0], \
                           [d_I_d_lssDiameter[2], 0]])

    def provideJ(self):

        input_keys = ['lssDiameter', 'rotorDiameter']
        output_keys = ['mass', 'cm[0]', 'cm[1]', 'cm[2]', 'I[0]', 'I[1]', 'I[2]']

        self.derivatives.set_first_derivative(input_keys, output_keys, self.J)

#-------------------------------------------------------------------------------

class Gearbox(Component):  
    ''' Gearbox class          
          The Gearbox class is used to represent the gearbox component of a wind turbine drivetrain.
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.           
    '''

    # variables
    rotorDiameter = Float(iotype='in', units='m', desc='rotor diameter')
    rotorTorque = Float(iotype='in', units='N*m', desc='rotor torque at rated power')
    gearRatio = Float(iotype='in', desc='overall gearbox ratio')

    # parameters
    drivetrainDesign = Int(iotype='in', desc='type of gearbox based on drivetrain type: 1 = standard 3-stage gearbox, 2 = single-stage, 3 = multi-gen, 4 = direct drive')
    gearConfiguration = Str(iotype='in', desc='string that represents the configuration of the gearbox (stage number and types)')
    bevel = Int(iotype='in', desc='Flag for the presence of a bevel stage - 1 if present, 0 if not')
    
    # returns
    stageMasses = Array(np.array([None, 0.0, 0.0, 0.0, 0.0]), iotype='out', desc='individual gearbox stage masses')
    mass = Float(0.0, iotype='out', units='kg', desc='overall component mass')
    cm = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system')
    I = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass')    
    
    def __init__(self):
        ''' 
        Initializes gearbox component 
        
        Parameters
        ----------
        rotorDiameter : float
          The wind turbine rotor diameter [m]
        rotorTorque : float
          The input torque from the wind turbine at rated power after accounting for drivetrain losses [N*m]
        gearRatio : float
          Overall gear ratio of the gearbox
        drivetrainDesign : int
          type of gearbox based on drivetrain type: 1 = standard 3-stage gearbox, 2 = single-stage, 3 = multi-gen, 4 = direct drive
        gearConfiguration : string
          string that represents the configuration of the gearbox (stage number and types)
        bevel : int
          Flag for the presence of a bevel stage - 1 if present, 0 if not

        Returns
        -------
        stageMasses : array of float
          individual stage mass of the gearbox [kg]
        mass : float
          mass of the component [kg]
        cm : array of float
          center of mass of the component (relative to tower top center) [m, m, m]
        I : array of float
          principle moments of inertia for the component (Ixx, iyy, Izz) around its center of mass (relative to tower top center) [kg*m^2, kg*m^2, kg*m^2]  
        '''

        super(Gearbox,self).__init__()
    
    def execute(self):
        
        self.mass = self.calc_mass([self.rotorDiameter, self.rotorTorque, self.gearRatio]) # [self.mass, self.cm[0], self.cm[1], self.cm[2], self.I[0], self.I[1], self.I[2]]

        # calculate mass properties
        cm0   = 0.0
        cm1   = cm0
        cm2   = 0.025 * self.rotorDiameter
        self.cm = np.array([cm0, cm1, cm2])

        length = (0.012 * self.rotorDiameter)
        height = (0.015 * self.rotorDiameter)
        diameter = (0.75 * height)

        I0 = self.mass * (diameter ** 2 ) / 8 + (self.mass / 2) * (height ** 2) / 8
        I1 = self.mass * (0.5 * (diameter ** 2) + (2 / 3) * (length ** 2) + 0.25 * (height ** 2)) / 8
        I2 = I1
        self.I = np.array([I0, I1, I2])
        
        # derviatives
        x = algopy.UTPM.init_jacobian([self.rotorDiameter, self.rotorTorque, self.gearRatio])  

        d_mass = algopy.UTPM.extract_jacobian(self.calc_mass(x))
        d_mass_d_rotorTorque = d_mass[1]
        d_mass_d_gearRatio = d_mass[2]

        d_cm_d_rotorDiameter = np.array([0.0, 0.0, 0.025])

        d_I_d_rotorDiameter = np.array([0.0, 0.0, 0.0])
        d_I_d_rotorDiameter[0] = (1/8) * self.mass * (2 * diameter * (0.75 * 0.015) + 0.5 * (2 * height * 0.015))
        d_I_d_rotorDiameter[1] = (1/8) * self.mass * ((0.5 * (2 * diameter) * (0.75 * 0.015)) + \
                                 (2/3) * (2 * length) * 0.012 + 0.25 * (2 * height) * 0.015)
        d_I_d_rotorDiameter[2] = d_I_d_rotorDiameter[1]

        d_I_d_rotorTorque = np.array([0.0, 0.0, 0.0])
        d_I_d_rotorTorque[0] = d_mass_d_rotorTorque * (diameter ** 2 ) / 8 + (self.mass / 2) * (height ** 2) / 8
        d_I_d_rotorTorque[1] = d_mass_d_rotorTorque * (0.5 * (diameter ** 2) + (2 / 3) * (length ** 2) + 0.25 * (height ** 2)) / 8
        d_I_d_rotorTorque[2] = d_I_d_rotorTorque[1]

        d_I_d_gearRatio = np.array([0.0, 0.0, 0.0])
        d_I_d_gearRatio[0] = d_mass_d_gearRatio * (diameter ** 2 ) / 8 + (self.mass / 2) * (height ** 2) / 8
        d_I_d_gearRatio[1] = d_mass_d_gearRatio * (0.5 * (diameter ** 2) + (2 / 3) * (length ** 2) + 0.25 * (height ** 2)) / 8
        d_I_d_gearRatio[2] = d_I_d_gearRatio[1]        
        
        # Jacobian
        self.J = np.array([[0, d_mass_d_rotorTorque, d_mass_d_gearRatio], \
                           [d_cm_d_rotorDiameter[0], 0, 0], \
                           [d_cm_d_rotorDiameter[1], 0, 0], \
                           [d_cm_d_rotorDiameter[2], 0, 0], \
                           [d_I_d_rotorDiameter[0], d_I_d_rotorTorque[0], d_I_d_gearRatio[0]], \
                           [d_I_d_rotorDiameter[1], d_I_d_rotorTorque[1], d_I_d_gearRatio[1]], \
                           [d_I_d_rotorDiameter[2], d_I_d_rotorTorque[2], d_I_d_gearRatio[2]]]) 

        # TODO: hack for second stage mass calcuation
        self.mass = self.calc_mass([self.rotorDiameter, self.rotorTorque, self.gearRatio]) # [self.mass, self.cm[0], self.cm[1], self.cm[2], self.I[0], self.I[1], self.I[2]]


    def calc_mass(self, x):
    	
        # inputs
        [rotorDiameter, rotorTorque, gearRatio] = x

        # compute masses, dimensions and cost
        overallweightFact = 1.00                          # default weight factor 1.0 (should depend on drivetrain design)
        self.stageMasses = [None, 0.0, 0.0, 0.0, 0.0]       # TODO: problem initializing stageMasses and accessing in compute
 
        # find weight of each stage depending on configuration
        # Gear ratio reduced for each stage based on principle that mixed epicyclic/parallel trains have a final stage ratio set at 1:2.5
        if self.gearConfiguration == 'p':
            self.stageMasses[1] = self.__getParallelStageWeight(rotorTorque,gearRatio)
        if self.gearConfiguration == 'e':
            self.stageMasses[1] = self.__getEpicyclicStageWeight(rotorTorque,gearRatio)
        if self.gearConfiguration == 'pp':
            self.stageMasses[1] = self.__getParallelStageWeight(rotorTorque,gearRatio**0.5)
            self.stageMasses[2] = self.__getParallelStageWeight(rotorTorque,gearRatio**0.5)
        if self.gearConfiguration == 'ep':
            self.stageMasses[1] = self.__getEpicyclicStageWeight(rotorTorque,gearRatio/2.5)
            self.stageMasses[2] = self.__getParallelStageWeight(rotorTorque,2.5)
        if self.gearConfiguration == 'ee':
            self.stageMasses[1] = self.__getEpicyclicStageWeight(rotorTorque,gearRatio**0.5)
            self.stageMasses[2] = self.__getEpicyclicStageWeight(rotorTorque,gearRatio**0.5)
        if self.gearConfiguration == 'eep':
            U1 = (gearRatio/3.0)**0.5
            U2 = (gearRatio/3.0)**0.5
            U3 = 3.0
            self.stageMasses[1] = self.__getEpicyclicStageWeight(rotorTorque,1,U1,U2,U3)  #different than sunderland
            self.stageMasses[2] = self.__getEpicyclicStageWeight(rotorTorque,2,U1,U2,U3)
            self.stageMasses[3] = self.__getParallelStageWeight(rotorTorque,3,U1,U2,U3)
        if self.gearConfiguration == 'epp':
            U1 = gearRatio**0.33*1.4
            U2 = (gearRatio**0.33/1.18)
            U3 = (gearRatio**0.33/1.18)
            self.stageMasses[1] = self.__getEpicyclicStageWeight(rotorTorque,1,U1,U2,U3)    # not in sunderland
            self.stageMasses[2] = self.__getParallelStageWeight(rotorTorque,2,U1,U2,U3)
            self.stageMasses[3] = self.__getParallelStageWeight(rotorTorque,3,U1,U2,U3)
        if self.gearConfiguration == 'eee':
            self.stageMasses[1] = self.__getEpicyclicStageWeight(rotorTorque,gearRatio**(0.33))
            self.stageMasses[2] = self.__getEpicyclicStageWeight(rotorTorque,gearRatio**(0.33))
            self.stageMasses[3] = self.__getEpicyclicStageWeight(rotorTorque,gearRatio**(0.33))
        if self.gearConfiguration == 'ppp':
            self.stageMasses[1] = self.__getParallelStageWeight(rotorTorque,gearRatio**(0.33))
            self.stageMasses[2] = self.__getParallelStageWeight(rotorTorque,gearRatio**(0.33))
            self.stageMasses[3] = self.__getParallelStageWeight(rotorTorque,gearRatio**(0.33))


        if (self.bevel):
            self.stageMasses[4] = 0.0454 * (rotorTorque ** 0.85)

        mass = 0.0
        for i in range(1,4):
            mass += self.stageMasses[i]
        mass     *= overallweightFact

        return mass

    def provideJ(self):

        input_keys = ['rotorDiameter', 'rotorTorque', 'gearRatio']
        output_keys = ['mass', 'cm[0]', 'cm[1]', 'cm[2]', 'I[0]', 'I[1]', 'I[2]']

        self.derivatives.set_first_derivative(input_keys, output_keys, self.J)

    def __getParallelStageWeight(self,rotorTorque,stage,stageRatio1,stageRatio2,stageRatio3):

        ''' 
          This method calculates the stage weight for a parallel stage in a gearbox based on the input torque, stage number, and stage ratio for each individual stage.
        '''
        
        serviceFact     = 1.00                                # default service factor for a gear stage is 1.75 based on full span VP (should depend on control type)
        applicationFact = 0.4                             # application factor ???        
        stageweightFact = 8.029 #/2                           # stage weight factor applied to each Gearbox stage

        if (rotorTorque * serviceFact) < 200000.0:       # design factor for design and manufacture of Gearbox
            designFact = 925.0
        elif (rotorTorque * serviceFact) < 700000.0:
            designFact = 1000.0
        else:
            designFact = 1100.0                            # TODO: should be an exception for all 2 stage Gearboxes to have designFact = 1000
            
        if stage == 1:
            Qr         = rotorTorque
            stageRatio = stageRatio1
        elif stage == 2:
            Qr         = rotorTorque/stageRatio1
            stageRatio = stageRatio2
        elif stage == 3:
            Qr         = rotorTorque/(stageRatio1*stageRatio2)
            stageRatio = stageRatio3

        gearFact = applicationFact / designFact          # Gearbox factor for design, manufacture and application of Gearbox
        
        gearweightFact = 1 + (1 / stageRatio) + stageRatio + (stageRatio ** 2)
                                                         # Gearbox weight factor for relationship of stage ratio required and relative stage volume

        stageWeight = stageweightFact * Qr * serviceFact * gearFact * gearweightFact
                                                         # forumula for parallel gearstage weight based on sunderland model

        return stageWeight

    def __getEpicyclicStageWeight(self,rotorTorque,stage,stageRatio1,stageRatio2,stageRatio3):
        ''' 
          This method calculates the stage weight for a epicyclic stage in a gearbox based on the input torque, stage number, and stage ratio for each individual stage
        '''

        serviceFact     = 1.00                                # default service factor for a gear stage is 1.75 based on full span VP (should depend on control type)
        applicationFact = 0.4                             # application factor ???        
        stageweightFact = 8.029/12                          # stage weight factor applied to each Gearbox stage
        OptWheels       = 3.0                                    # default optional wheels (should depend on stage design)

        if (rotorTorque * serviceFact) < 200000.0:       # design factor for design and manufacture of Gearbox
            designFact = 850.0
        elif (rotorTorque * serviceFact) < 700000.0:
            designFact = 950.0
        else:
            designFact = 1100.0
           
        if stage == 1:
            Qr         = rotorTorque
            stageRatio = stageRatio1
        elif stage == 2:
            Qr         = rotorTorque/stageRatio1
            stageRatio = stageRatio2
        elif stage == 3:
            Qr         = rotorTorque/(stageRatio1*stageRatio2)
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
    rotorDiameter = Float(iotype='in', units='m', desc='rotor diameter')
    rotorTorque = Float(iotype='in', units='N*m', desc='rotor torque at rated power')
    gearRatio = Float(iotype='in', desc='overall gearbox ratio')
    lssDiameter = Float(iotype='in', units='m', desc='low speed shaft outer diameter')
    
    # returns
    mass = Float(0.0, iotype='out', units='kg', desc='overall component mass')
    cm = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system')
    I = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass')   

    def __init__(self):
        ''' 
        Initializes high speed side component 
        
        Parameters
        ----------
        rotorDiameter : float
          The wind turbine rotor diameter [m]
        rotorTorque : float
          The input torque from the wind turbine at rated power after accounting for drivetrain losses [N*m]
        gearRatio : float
          Overall gear ratio of the gearbox
        lssDiameter : float
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

        super(HighSpeedSide,self).__init__()
    
    def execute(self):
        
        # compute masses, dimensions and cost        
        designTorque = self.rotorTorque / self.gearRatio               # design torque [Nm] based on rotor torque and Gearbox ratio
        massFact = 0.025                                 # mass matching factor default value
        highSpeedShaftMass = (massFact * designTorque)
        
        mechBrakeMass = (0.5 * highSpeedShaftMass)      # relationship derived from HSS multiplier for University of Sunderland model compared to NREL CSM for 750 kW and 1.5 MW turbines

        self.mass = (mechBrakeMass + highSpeedShaftMass)                     

        # calculate mass properties
        cm = np.array([0.0,0.0,0.0])
        cm[0]   = 0.5 * (0.0125 * self.rotorDiameter)
        cm[1]   = 0.0
        cm[2]   = 0.025 * self.rotorDiameter
        self.cm = cm

        diameter = (1.5 * self.lssDiameter)                     # based on WindPACT relationships for full HSS / mechanical brake assembly
        length = (0.025)
        matlDensity = 7850 # material density kg/m^3

        I = np.array([0.0, 0.0, 0.0])
        I[0]    = 0.25 * length * 3.14159 * matlDensity * (diameter ** 2) * (self.gearRatio**2) * (diameter ** 2) / 8
        I[1]    = self.mass * ((3/4) * (diameter ** 2) + (length ** 2)) / 12
        I[2]    = I[1]      
        self.I = I

        # derivatives
        d_mass_d_rotorTorque = 1.5 * massFact / self.gearRatio
        d_mass_d_gearRatio = 1.5 * massFact * self.rotorTorque * (-1 / (self.gearRatio**2))
        
        d_cm_d_rotorDiameter = np.array([0.5 * 0.0125, 0.0, 0.025])
        
        d_I_d_lssDiameter = np.array([0.0, 0.0, 0.0])
        d_I_d_lssDiameter[0] = 4 * 1.5 * (0.25 * length * 3.14159 * matlDensity * (self.gearRatio**2) * (diameter ** 3) / 8)
        d_I_d_lssDiameter[1] = 2 * 1.5 * (self.mass * ((3/4) * (diameter) + (length ** 2)) / 12)
        d_I_d_lssDiameter[2] = d_I_d_lssDiameter[1]
        
        d_I_d_gearRatio = np.array([0.0, 0.0, 0.0])
        d_I_d_gearRatio[0] = 2 * (0.25 * length * 3.14159 * matlDensity * (diameter ** 2) * (self.gearRatio) * (diameter ** 2) / 8)
        d_I_d_gearRatio[1] = d_mass_d_gearRatio * ((3/4) * (diameter ** 2) + (length ** 2)) / 12
        d_I_d_gearRatio[2] = d_I_d_gearRatio[1]
        
        d_I_d_rotorTorque = np.array([0.0, 0.0, 0.0])
        d_I_d_rotorTorque[1] = d_mass_d_rotorTorque * ((3/4) * (diameter ** 2) + (length ** 2)) / 12
        d_I_d_rotorTorque[2] = d_I_d_rotorTorque[1]        
        
        # Jacobian
        self.J = np.array([[0, d_mass_d_rotorTorque, d_mass_d_gearRatio, 0], \
                           [d_cm_d_rotorDiameter[0], 0, 0, 0], \
                           [d_cm_d_rotorDiameter[1], 0, 0, 0], \
                           [d_cm_d_rotorDiameter[2], 0, 0, 0], \
                           [0, d_I_d_lssDiameter[0], d_I_d_rotorTorque[0], d_I_d_gearRatio[0]], \
                           [0, d_I_d_lssDiameter[1], d_I_d_rotorTorque[1], d_I_d_gearRatio[1]], \
                           [0, d_I_d_lssDiameter[2], d_I_d_rotorTorque[2], d_I_d_gearRatio[2]]])

    def provideJ(self):

        input_keys = ['rotorDiameter','lssDiameter', 'rotorTorque', 'gearRatio']
        output_keys = ['mass', 'cm[0]', 'cm[1]', 'cm[2]', 'I[0]', 'I[1]', 'I[2]']

        self.derivatives.set_first_derivative(input_keys, output_keys, self.J)


#-------------------------------------------------------------------------------

class Generator(Component):
    '''Generator class          
          The Generator class is used to represent the generator of a wind turbine drivetrain.
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.           
    '''

    # variables
    rotorDiameter = Float(iotype='in', units='m', desc='rotor diameter')
    machineRating = Float(iotype='in', units='kW', desc='machine rating of generator')
    gearRatio = Float(iotype='in', desc='overall gearbox ratio')

    # parameters
    drivetrainDesign = Int(iotype='in', desc='type of gearbox based on drivetrain type: 1 = standard 3-stage gearbox, 2 = single-stage, 3 = multi-gen, 4 = direct drive')
    
    # returns
    mass = Float(0.0, iotype='out', units='kg', desc='overall component mass')
    cm = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system')
    I = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass')    
    
    def __init__(self):
        ''' 
        Initializes generator component 
        
        Parameters
        ----------
        rotorDiameter : float
          The wind turbine rotor diameter [m]
        machineRating : float
          Turbine machine rating [kW]
        gearRatio : float
          Overall gear ratio of the gearbox
        drivetrainDesign : int
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
    
    def execute(self):

        massCoeff = [None, 6.4737, 10.51 ,  5.34  , 37.68  ]           
        massExp   = [None, 0.9223, 0.9223,  0.9223, 1      ]

        CalcRPM    = 80 / (self.rotorDiameter*0.5*pi/30)
        CalcTorque = (self.machineRating*1.1) / (CalcRPM * pi/30)
        
        if (self.drivetrainDesign < 4):
            self.mass = (massCoeff[self.drivetrainDesign] * self.machineRating ** massExp[self.drivetrainDesign])   
        else:  # direct drive
            self.mass = (massCoeff[self.drivetrainDesign] * CalcTorque ** massExp[self.drivetrainDesign])  

        # calculate mass properties
        cm = np.array([0.0,0.0,0.0])
        cm[0]  = 0.0125 * self.rotorDiameter
        cm[1]  = 0.0
        cm[2]  = 0.025 * self.rotorDiameter
        self.cm = cm

        length = (1.6 * 0.015 * self.rotorDiameter)
        depth = (0.015 * self.rotorDiameter)
        width = (0.5 * depth)

        I = np.array([0.0, 0.0, 0.0])
        I[0]   = ((4.86 * (10 ** (-5))) * (self.rotorDiameter ** 5.333)) + (((2/3) * self.mass) * (depth ** 2 + width ** 2) / 8)
        I[1]   = (I[0] / 2) / (self.gearRatio ** 2) + ((1/3) * self.mass * (length ** 2) / 12) + (((2 / 3) * self.mass) * \
                   (depth ** 2 + width ** 2 + (4/3) * (length ** 2)) / 16 )
        I[2]   = I[1]                           
        self.I = I

        # derivatives
        if (self.drivetrainDesign < 4):
            d_mass_d_rotorDiameter = 0   
        else:  # direct drive
            d_mass_d_rotorDiameter = massExp[self.drivetrainDesign] * (massCoeff[self.drivetrainDesign] * CalcTorque ** (massExp[self.drivetrainDesign] - 1)) * (self.machineRating * 1.1 * 0.5 / 80)

        if (self.drivetrainDesign < 4):
            d_mass_d_machineRating = massExp[self.drivetrainDesign] * (massCoeff[self.drivetrainDesign] * self.machineRating ** (massExp[self.drivetrainDesign]-1))  
        else:  # direct drive
            d_mass_d_machineRating = massExp[self.drivetrainDesign] * (massCoeff[self.drivetrainDesign] * CalcTorque ** (massExp[self.drivetrainDesign] - 1)) * (self.rotorDiameter * 1.1 * 0.5 / 80)
        
        d_cm_d_rotorDiameter = np.array([0.0125, 0.0, 0.025])
        
        d_I_d_rotorDiameter = np.array([0.0, 0.0, 0.0])
        d_I_d_rotorDiameter[0] = 5.333*(4.86*(10**-5))*(self.rotorDiameter**4.333) + \
                                (1/8) * (2/3) * d_mass_d_rotorDiameter * (depth ** 2 + width ** 2) + (1/8) * (2/3) * self.mass * (2*depth*0.015 + 2*width*0.5*0.15)
        d_I_d_rotorDiameter[1] = (1/(2*self.gearRatio**2))*(d_I_d_rotorDiameter[0]) + \
                                 d_mass_d_rotorDiameter * ((1/3) * (length ** 2) / 12) + (((2 / 3)) * (depth ** 2 + width ** 2 + (4/3) * (length ** 2)) / 16 ) + \
                                 self.mass * ((1/3) * (1/12) * (2 * length * 1.6 * 0.015) + (2/3) * (1/16) * (2*depth*0.015 + 2*width*0.5*0.15 + (4/3)*2*length*1.6*0.015))
        d_I_d_rotorDiameter[2] = d_I_d_rotorDiameter[1]

        d_I_d_machineRating = np.array([0.0, 0.0, 0.0])
        d_I_d_machineRating[0] = (1/8) * (2/3) * d_mass_d_machineRating * (2*depth*0.15 + 2*width*0.5*0.15)
        d_I_d_machineRating[1] = (1/(2*self.gearRatio**2))*d_I_d_machineRating[0]
        d_I_d_machineRating[2] = d_I_d_machineRating[1]
        
        d_I_d_gearRatio = np.array([0.0, 0.0, 0.0])
        d_I_d_gearRatio[1] = (1/2) * self.I[0] * (-1/(self.gearRatio**3))
        d_I_d_gearRatio[2] = d_I_d_gearRatio[1]
        
        # Jacobian
        self.J = np.array([[d_mass_d_rotorDiameter, d_mass_d_machineRating, 0], \
                           [d_cm_d_rotorDiameter[0], 0, 0], \
                           [d_cm_d_rotorDiameter[1], 0, 0], \
                           [d_cm_d_rotorDiameter[2]], \
                           [0, d_I_d_rotorDiameter[0], d_I_d_machineRating[0], d_I_d_gearRatio[0]], \
                           [0, d_I_d_rotorDiameter[1], d_I_d_machineRating[1], d_I_d_gearRatio[1]], \
                           [0, d_I_d_rotorDiameter[2], d_I_d_machineRating[2], d_I_d_gearRatio[2]]])

    def provideJ(self):

        input_keys = ['rotorDiameter','machineRating', 'gearRatio']
        output_keys = ['mass', 'cm[0]', 'cm[1]', 'cm[2]', 'I[0]', 'I[1]', 'I[2]']

        self.derivatives.set_first_derivative(input_keys, output_keys, self.J)

#-------------------------------------------------------------------------------

class Bedplate(Component):
    ''' Bedplate class         
          The Bedplate class is used to represent the bedplate of a wind turbine drivetrain.
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.           
    '''
    # variables
    rotorDiameter = Float(iotype='in', units='m', desc='rotor diameter')
    rotorThrust = Float(iotype='in', units='N', desc='maximum rotor thrust')
    rotorMass = Float(iotype='in', units='kg', desc='rotor overall mass')
    rotorTorque = Float(iotype='in', units='N*m', desc='rotor torque at rated power')
    towerTopDiameter = Float(iotype='in', units='m', desc='tower top diameter')
   
    # parameters
    drivetrainDesign = Int(iotype='in', desc='type of gearbox based on drivetrain type: 1 = standard 3-stage gearbox, 2 = single-stage, 3 = multi-gen, 4 = direct drive')
    
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
        rotorDiameter : float
          The wind turbine rotor diameter [m]
        rotorThrust : float
          Maximum thrust from the rotor applied to the drivetrain under extreme conditions [N]
        rotorMass : float
          Mass of the rotor [kg]
        rotorTorque : float
          The input torque from the wind turbine at rated power after accounting for drivetrain losses [N*m]
        towerTopDiameter : float
          Diameter of the turbine tower top [m] 
        drivetrainDesign : int
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

        super(Bedplate,self).__init__()

    def execute(self):

        # compute masses, dimensions and cost
        # bedplate sizing based on superposition of loads for rotor torque, thurst, weight         #TODO: only handles bedplate for a traditional drivetrain configuration
        bedplateWeightFact = 2.86                                   # toruqe weight factor for bedplate (should depend on drivetrain, bedplate type)

        torqueweightCoeff = 0.00368                   # regression coefficient multiplier for bedplate weight based on rotor torque
        MassFromTorque    = bedplateWeightFact * (torqueweightCoeff * self.rotorTorque)
                                                                  
        thrustweightCoeff = 0.00158                                 # regression coefficient multiplier for bedplate weight based on rotor thrust
        MassFromThrust    = bedplateWeightFact * (thrustweightCoeff * (self.rotorThrust * self.towerTopDiameter))

        rotorweightCoeff    = 0.015                                    # regression coefficient multiplier for bedplate weight based on rotor weight
        MassFromRotorWeight = bedplateWeightFact * (rotorweightCoeff * (self.rotorMass * self.towerTopDiameter))
        
        # additional weight ascribed to bedplate area
        BPlengthFact    = 1.5874                                       # bedplate length factor (should depend on drivetrain, bedplate type)
        nacellevolFact  = 0.052                                      # nacelle volume factor (should depend on drivetrain, bedplate type)
        self.length = (BPlengthFact * nacellevolFact * self.rotorDiameter)     # bedplate length [m] calculated as a function of rotor diameter
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

        self.mass = (massCoeff[self.drivetrainDesign] * self.rotorDiameter ** massExp[self.drivetrainDesign] )
        
        # calculate mass properties
        cm = np.array([0.0,0.0,0.0])
        cm[0] = cm[1] = 0.0
        cm[2] = 0.0122 * self.rotorDiameter                             # half distance from shaft to yaw axis
        self.cm = cm

        depth = (self.length / 2.0)

        I = np.array([0.0, 0.0, 0.0])
        I[0]  = self.mass * (self.width ** 2 + depth ** 2) / 8
        I[1]  = self.mass * (depth ** 2 + self.width ** 2 + (4/3) * self.length ** 2) / 16
        I[2]  = I[1]                          
        self.I = I

        # derivatives
        d_length_d_rotorDiameter = BPlengthFact * nacellevolFact
        d_width_d_rotorDiameter = (1/2.0) * d_length_d_rotorDiameter
        d_depth_d_rotorDiameter = (1/2.0) * d_length_d_rotorDiameter
        
        d_mass_d_rotorDiameter = bedplateWeightFact * areaweightCoeff * ((BPlengthFact * nacellevolFact)**2) * self.rotorDiameter
        d_mass_d_rotorThrust = bedplateWeightFact * (thrustweightCoeff * (self.towerTopDiameter))
        d_mass_d_rotorTorque = bedplateWeightFact * torqueweightCoeff
        d_mass_d_rotorMass = bedplateWeightFact * (rotorweightCoeff * (self.towerTopDiameter))
        d_mass_d_towerTopDiameter = bedplateWeightFact * (thrustweightCoeff * (self.rotorThrust)) + bedplateWeightFact * (rotorweightCoeff * (self.rotorMass))
        
        d_cm_d_rotorDiameter = np.array([0.0, 0.0, 0.0122])

        d_I_d_rotorDiameter = np.array([0.0, 0.0, 0.0])        
        d_I_d_rotorDiameter[0] = (1/8) * (d_mass_d_rotorDiameter)*(self.width**2 + depth**2) + (1/8) * self.mass * (2*self.width*d_width_d_rotorDiameter + 2*depth*d_depth_d_rotorDiameter)
        d_I_d_rotorDiameter[1] = (1/8) * (d_mass_d_rotorDiameter)*(self.width**2 + depth**2 + (4/3) * self.length**2) + (1/8) * self.mass * (2*self.width*d_width_d_rotorDiameter + 2*depth*d_depth_d_rotorDiameter + (4/3)*2*self.length*d_length_d_rotorDiameter)
        d_I_d_rotorDiameter[2] = d_I_d_rotorDiameter[1]

        d_I_d_rotorThrust = np.array([0.0, 0.0, 0.0])        
        d_I_d_rotorThrust[0] = (1/8) * (d_mass_d_rotorThrust)*(self.width**2 + depth**2)
        d_I_d_rotorThrust[1] = (1/8) * (d_mass_d_rotorThrust)*(self.width**2 + depth**2 + (4/3) * self.length**2)
        d_I_d_rotorThrust[2] = d_I_d_rotorThrust[1]

        d_I_d_rotorTorque = np.array([0.0, 0.0, 0.0])        
        d_I_d_rotorTorque[0] = (1/8) * (d_mass_d_rotorTorque)*(self.width**2 + depth**2)
        d_I_d_rotorTorque[1] = (1/8) * (d_mass_d_rotorTorque)*(self.width**2 + depth**2 + (4/3) * self.length**2)
        d_I_d_rotorTorque[2] = d_I_d_rotorTorque[1]       
        
        d_I_d_rotorMass = np.array([0.0, 0.0, 0.0])        
        d_I_d_rotorMass[0] = (1/8) * (d_mass_d_rotorMass)*(self.width**2 + depth**2)
        d_I_d_rotorMass[1] = (1/8) * (d_mass_d_rotorMass)*(self.width**2 + depth**2 + (4/3) * self.length**2)
        d_I_d_rotorMass[2] = d_I_d_rotorMass[1] 
        
        d_I_d_towerTopDiameter = np.array([0.0, 0.0, 0.0])        
        d_I_d_towerTopDiameter[0] = (1/8) * (d_mass_d_towerTopDiameter)*(self.width**2 + depth**2)
        d_I_d_towerTopDiameter[1] = (1/8) * (d_mass_d_towerTopDiameter)*(self.width**2 + depth**2 + (4/3) * self.length**2)
        d_I_d_towerTopDiameter[2] = d_I_d_towerTopDiameter[1]

        # Jacobian
        self.J = np.array([[d_mass_d_rotorDiameter, d_mass_d_rotorThrust, d_mass_d_rotorTorque, d_mass_d_rotorMass, d_mass_d_towerTopDiameter], \
                           [d_cm_d_rotorDiameter[0], 0, 0, 0, 0], \
                           [d_cm_d_rotorDiameter[1], 0, 0, 0, 0], \
                           [d_cm_d_rotorDiameter[2], 0, 0, 0, 0], \
                           [d_I_d_rotorDiameter[0], d_I_d_rotorThrust[0], d_I_d_rotorTorque[0], d_I_d_rotorMass[0], d_I_d_towerTopDiameter[0]], \
                           [d_I_d_rotorDiameter[1], d_I_d_rotorThrust[1], d_I_d_rotorTorque[1], d_I_d_rotorMass[1], d_I_d_towerTopDiameter[1]], \
                           [d_I_d_rotorDiameter[2], d_I_d_rotorThrust[2], d_I_d_rotorTorque[2], d_I_d_rotorMass[2], d_I_d_towerTopDiameter[2]], \
                           [d_length_d_rotorDiameter, 0, 0, 0, 0], \
                           [d_width_d_rotorDiameter, 0, 0, 0, 0]])

    def provideJ(self):

        input_keys = ['rotorDiameter','rotorThrust', 'rotorTorque', 'rotorMass', 'towerTopDiameter']
        output_keys = ['mass', 'cm[0]', 'cm[1]', 'cm[2]', 'I[0]', 'I[1]', 'I[2]', 'length', 'width']

        self.derivatives.set_first_derivative(input_keys, output_keys, self.J)
                           

#-------------------------------------------------------------------------------

class AboveYawMassAdder(Component):

    # variables
    machineRating = Float(iotype = 'in', units='kW', desc='machine rating')
    lowSpeedShaftMass = Float(iotype = 'in', units='kg', desc='component mass')
    mainBearingMass = Float(iotype = 'in', units='kg', desc='component mass')
    secondBearingMass = Float(iotype = 'in', units='kg', desc='component mass')
    gearboxMass = Float(iotype = 'in', units='kg', desc='component mass')
    highSpeedSideMass = Float(iotype = 'in', units='kg', desc='component mass')
    generatorMass = Float(iotype = 'in', units='kg', desc='component mass')
    bedplateMass = Float(iotype = 'in', units='kg', desc='component mass')
    bedplateLength = Float(iotype = 'in', units='m', desc='component length')
    bedplateWidth = Float(iotype = 'in', units='m', desc='component width')

    # parameters
    crane = Bool(iotype='in', desc='flag for presence of crane')

    # returns    
    econnectionsMass = Float(iotype = 'out', units='kg', desc='component mass')         
    vspdEtronicsMass = Float(iotype = 'out', units='kg', desc='component mass')         
    hydrCoolingMass = Float(iotype = 'out', units='kg', desc='component mass')              
    controlsMass = Float(iotype = 'out', units='kg', desc='component mass')         
    nacellePlatformsMass = Float(iotype = 'out', units='kg', desc='component mass')            
    craneMass = Float(iotype = 'out', units='kg', desc='component mass')                   
    mainframeMass = Float(iotype = 'out', units='kg', desc='component mass')               
    nacelleCovMass = Float(iotype = 'out', units='kg', desc='component mass')
    aboveYawMass = Float(iotype = 'out', units='kg', desc='total mass above yaw system')
    length = Float(iotype = 'out', units='m', desc='component length')
    width = Float(iotype = 'out', units='m', desc='component width')
    height = Float(iotype = 'out', units='m', desc='component height')
    
    def __init__(self):
        ''' Initialize above yaw mass adder component
        
        Parameters
        ----------
		    machineRating : float
		      machine rating [kW]
		    lowSpeedShaftMass : float
		      low speed shaft component mass [kg]
		    mainBearingMass : float
		      main bearing component mass [kg]
		    secondBearingMass : float
		      second bearing component mass [kg]
		    gearboxMass : float
		      gearbox component mass [kg]
		    highSpeedSideMass : float
		      high speed side component mass [kg]
		    generatorMass : float
		      generator component mass [kg]
		    bedplateMass : float
		      bedplate component mass [kg]
		    bedplateLength : float
		      bedplate component length [m]
		    bedplateWidth : float
		      bedplate component width [m]
		    crane : boolean
		      flag for presence of crane
		
		    Returns
		    -------    
		    econnectionsMass : float
		      electrical connections component mass [kg]
		    vspdEtronicsMass : float
		      variable speed electrics mass [kg]    
		    hydrCoolingMass : float
		      HVAC component mass [kg]
		    controlsMass : float
		      controls component mass [kg]
		    nacellePlatformsMass : float
		      nacelle platforms component mass [kg]
		    craneMass : float
		      crane component mass [kg]
		    mainframeMass : float
		      mainframe component mass [kg]               
		    nacelleCovMass : float
		      nacelle cover component mass [kg]
		    aboveYawMass : float
		      total mass above yaw system [kg]
		    length : float
		      nacelle component length [m]
		    width : float
		      nacelle component width [m]
		    height : float
		      nacelle component height [m]     
        '''
    
        super(AboveYawMassAdder, self).__init__()

    def execute(self):

        # electronic systems, hydraulics and controls 
        self.econnectionsMass = 0.0

        self.vspdEtronicsMass = 2.4445*self.machineRating + 1599.0
               
        self.hydrCoolingMass = 0.08 * self.machineRating
 
        self.controlsMass     = 0.0

        # mainframe system including bedplate, platforms, crane and miscellaneous hardware
        self.nacellePlatformsMass = 0.125 * self.bedplateMass

        if (self.crane):
            self.craneMass =  3000.0
        else:
            self.craneMass = 0.0
 
        self.mainframeMass  = self.bedplateMass + self.craneMass + self.nacellePlatformsMass     
        
        nacelleCovArea      = 2 * (self.bedplateLength ** 2)              # this calculation is based on Sunderland
        self.nacelleCovMass = (84.1 * nacelleCovArea) / 2          # this calculation is based on Sunderland - divided by 2 in order to approach CSM

        # yaw system weight calculations based on total system mass above yaw system
        self.aboveYawMass =  self.lowSpeedShaftMass + \
                    self.mainBearingMass + self.secondBearingMass + \
                    self.gearboxMass + \
                    self.highSpeedSideMass + \
                    self.generatorMass + \
                    self.mainframeMass + \
                    self.econnectionsMass + \
                    self.vspdEtronicsMass + \
                    self.hydrCoolingMass + \
                    self.nacelleCovMass

        self.length      = self.bedplateLength                              # nacelle length [m] based on bedplate length
        self.width       = self.bedplateWidth                        # nacelle width [m] based on bedplate width
        self.height      = (2.0 / 3.0) * self.length                         # nacelle height [m] calculated based on cladding area
        
        # derivatives
        d_hydrCoolingMass_d_machineRating = 0.08
        d_nacellePlatformsMass_d_bedplateMass = 0.125
        d_mainframeMass_d_bedplateMass = 1.0 + d_nacellePlatformsMass_d_bedplateMass
        d_nacelleCovMass_d_bedplateLength = (84.1 / 2) * (2*2) * self.bedplateLength
        d_aboveYawMass_d_bedplateMass = d_mainframeMass_d_bedplateMass
        d_aboveYawMass_d_bedplateLength = d_nacelleCovMass_d_bedplateLength
        d_aboveYawMass_d_machineRating = d_hydrCoolingMass_d_machineRating
        d_length_d_bedplateLength = 1.0
        d_width_d_bedplateWidth = 1.0
        d_height_d_bedplateLength = (2.0/3.0)
        
        # Jacobian
        self.J = np.array([[d_hydrCoolingMass_d_machineRating, 0, 0, 0, 0, 0, 0, 0, 0, 0], \
                          [0, 0, 0, 0, 0, 0, 0, d_nacellePlatformsMass_d_bedplateMass, 0, 0], \
                          [0, 0, 0, 0, 0, 0, 0, d_mainframeMass_d_bedplateMass, 0, 0], \
                          [0, 0, 0, 0, 0, 0, 0, 0, d_nacelleCovMass_d_bedplateLength, 0], \
                          [d_aboveYawMass_d_machineRating, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, d_aboveYawMass_d_bedplateMass, d_aboveYawMass_d_bedplateLength, 0], \
                          [0, 0, 0, 0, 0, 0, 0, 0, d_length_d_bedplateLength, 0], \
                          [0, 0, 0, 0, 0, 0, 0, 0, 0, d_width_d_bedplateWidth], \
                          [0, 0, 0, 0, 0, 0, 0, 0, d_height_d_bedplateLength, 0]])

    def provideJ(self):

        input_keys = ['machineRating','lowSpeedShaftMass', 'mainBearingMass', 'secondBearingMass', 'gearboxMass', 'highSpeedSideMass', 'generatorMass', 'bedplateMass', 'bedplateLength', 'bedplateWidth']
        output_keys = ['hydrCoolingMass', 'nacellePlatformsMass', 'mainframeMass', 'nacelleCovMass', 'aboveYawMass', 'length', 'width', 'height']

        self.derivatives.set_first_derivative(input_keys, output_keys, self.J)

class YawSystem(Component):
    ''' YawSystem class
          The YawSystem class is used to represent the yaw system of a wind turbine drivetrain.
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.
    '''

    # variables
    rotorDiameter = Float(iotype='in', units='m', desc='rotor diameter')
    rotorThrust = Float(iotype='in', units='N', desc='maximum rotor thrust')
    towerTopDiameter = Float(iotype='in', units='m', desc='tower top diameter')
    aboveYawMass = Float(iotype='in', units='kg', desc='above yaw mass')
    
    # returns
    mass = Float(0.0, iotype='out', units='kg', desc='overall component mass')
    cm = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system')
    I = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass') 
 
    def __init__(self):
        ''' 
        Initializes yaw system component 
        
        Parameters
        ----------
        rotorDiameter : float
          The wind turbine rotor diameter [m]
        rotorThrust : float
          Maximum thrust from the rotor applied to the drivetrain under extreme conditions [N]
        towerTopDiameter : float
          Diameter of the turbine tower top [m] 
        aboveYawMass : float
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
    
    def execute(self): 
        
        # yaw weight depends on moment due to weight of components above yaw bearing and moment due to max thrust load
        #aboveYawMass = 350000 # verboseging number based on 5 MW RNA mass
        yawfactor = 0.41 * (2.4 * (10 ** (-3)))                   # should depend on rotor configuration: blade number and hub type
        weightMom = self.aboveYawMass * self.rotorDiameter                    # moment due to weight above yaw system
        thrustMom = self.rotorThrust * self.towerTopDiameter                  # moment due to rotor thrust
        self.mass = (yawfactor * (0.4 * weightMom + 0.975 * thrustMom))
        
        # calculate mass properties
        # yaw system assumed to be collocated to tower top center              
        cm = np.array([0.0,0.0,0.0])
        self.cm = cm

        I = np.array([0.0, 0.0, 0.0])
        self.I = I
        
        # derivatives
        d_mass_d_rotorDiameter = yawfactor * 0.4 * self.aboveYawMass
        d_mass_d_rotorThrust = yawfactor * 0.975 * self.towerTopDiameter
        d_mass_d_towerTopDiameter = yawfactor * 0.4 * self.rotorDiameter
        d_mass_d_aboveYawMass = yawfactor * 0.975 * self.rotorThrust
        
        # Jacobian
        self.J = np.array([d_mass_d_rotorDiameter, d_mass_d_rotorThrust, d_mass_d_towerTopDiameter, d_mass_d_aboveYawMass])
        
    def provideJ(self):

        input_keys = ['rotorDiameter', 'rotorThrust', 'towerTopDiameter', 'aboveYawMass']
        output_keys = ['mass']

        self.derivatives.set_first_derivative(input_keys, output_keys, self.J)    

#-------------------------------------------------------------------------------

class NacelleSystemAdder(Component): # changed name to nacelle - need to rename, move code pieces, develop configurations ***
    ''' NacelleSystem class       
          The Nacelle class is used to represent the overall nacelle of a wind turbine.
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.
    '''
    # variables
    aboveYawMass = Float(iotype='in', units='kg', desc='mass above yaw system')
    yawMass = Float(iotype='in', units='kg', desc='mass of yaw system')
    lowSpeedShaftMass = Float(iotype = 'in', units='kg', desc='component mass')
    mainBearingMass = Float(iotype = 'in', units='kg', desc='component mass')
    secondBearingMass = Float(iotype = 'in', units='kg', desc='component mass')
    gearboxMass = Float(iotype = 'in', units='kg', desc='component mass')
    highSpeedSideMass = Float(iotype = 'in', units='kg', desc='component mass')
    generatorMass = Float(iotype = 'in', units='kg', desc='component mass')
    bedplateMass = Float(iotype = 'in', units='kg', desc='component mass')
    mainframeMass = Float(iotype = 'in', units='kg', desc='component mass')
    lowSpeedShaftCM = Array(np.array([0.0,0.0,0.0]),iotype = 'in', units='kg', desc='component CM')
    mainBearingCM = Array(np.array([0.0,0.0,0.0]),iotype = 'in', units='kg', desc='component CM')
    secondBearingCM = Array(np.array([0.0,0.0,0.0]),iotype = 'in', units='kg', desc='component CM')
    gearboxCM = Array(np.array([0.0,0.0,0.0]),iotype = 'in', units='kg', desc='component CM')
    highSpeedSideCM = Array(np.array([0.0,0.0,0.0]),iotype = 'in', units='kg', desc='component CM')
    generatorCM = Array(np.array([0.0,0.0,0.0]),iotype = 'in', units='kg', desc='component CM')
    bedplateCM = Array(np.array([0.0,0.0,0.0]),iotype = 'in', units='kg', desc='component CM')
    lowSpeedShaftI = Array(np.array([0.0,0.0,0.0]),iotype = 'in', units='kg', desc='component I')
    mainBearingI = Array(np.array([0.0,0.0,0.0]),iotype = 'in', units='kg', desc='component I')
    secondBearingI = Array(np.array([0.0,0.0,0.0]),iotype = 'in', units='kg', desc='component I')
    gearboxI = Array(np.array([0.0,0.0,0.0]),iotype = 'in', units='kg', desc='component I')
    highSpeedSideI = Array(np.array([0.0,0.0,0.0]),iotype = 'in', units='kg', desc='component I')
    generatorI = Array(np.array([0.0,0.0,0.0]),iotype = 'in', units='kg', desc='component I')
    bedplateI = Array(np.array([0.0,0.0,0.0]),iotype = 'in', units='kg', desc='component I')

    # returns      
    mass = Float(0.0, iotype='out', units='kg', desc='overall component mass')
    cm = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system')
    I = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass') 

    def __init__(self):
        ''' Initialize above yaw mass adder component
        
        Parameters
        ----------
		    aboveYawMass : float
		      total mass above yaw system [kg]
		    yawMass : float
		      mass of yaw component [kg]
		    lowSpeedShaftMass : float
		      low speed shaft component mass [kg]
		    mainBearingMass : float
		      main bearing component mass [kg]
		    secondBearingMass : float
		      second bearing component mass [kg]
		    gearboxMass : float
		      gearbox component mass [kg]
		    highSpeedSideMass : float
		      high speed side component mass [kg]
		    generatorMass : float
		      generator component mass [kg]
		    bedplateMass : float
		      bedplate component mass [kg]
		    mainframeMass : float
		      mainframe component mass [kg]    
		    lowSpeedShaftCM : array of float
		      low speed shaft component cm [m, m, m]
		    mainBearingCM : array of float
		      main bearing component cm [m, m, m]
		    secondBearingCM : array of float
		      second bearing component cm [m, m, m]
		    gearboxCM : array of float
		      gearbox component cm [m, m, m]
		    highSpeedSideCM : array of float
		      high speed side component cm [m, m, m]
		    generatorCM : array of float
		      generator component cm [m, m, m]
		    bedplateCM : array of float
		      bedplate component cm [m, m, m]
		    lowSpeedShaftI : array of float
		      low speed shaft component mass moments of inertia for principle axis
		    mainBearingI : array of float
		      main bearing component mass moments of inertia for principle axis
		    secondBearingI : array of float
		      second bearing component mass moments of inertia for principle axis
		    gearboxI : array of float
		      gearbox component mass moments of inertia for principle axis
		    highSpeedSideI : array of float
		      high speed side component mass moments of inertia for principle axis
		    generatorI : array of float
		      generator component mass moments of inertia for principle axis
		    bedplateI : array of float
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
    
    def execute(self):

        # aggregation of nacelle mass
        self.mass = (self.aboveYawMass + self.yawMass)

        # calculation of mass center and moments of inertia
        cm = np.array([0.0,0.0,0.0])
        for i in (range(0,3)):
            # calculate center of mass (use mainframeMass in place of bedplateMass - assume lumped around bedplateCM)
            cm[i] = (self.lowSpeedShaftMass * self.lowSpeedShaftCM[i] + 
                    self.mainBearingMass * self.mainBearingCM[i] + self.secondBearingMass * self.secondBearingCM[i] + \
                    self.gearboxMass * self.gearboxCM[i] + self.highSpeedSideMass * self.highSpeedSideCM[i] + \
                    self.generatorMass * self.generatorCM[i] + self.mainframeMass * self.bedplateCM[i] ) / \
                    (self.lowSpeedShaftMass + self.mainBearingMass + self.secondBearingMass + \
                    self.gearboxMass + self.highSpeedSideMass + self.generatorMass + self.mainframeMass)
        self.cm = cm

        I = np.array([0.0, 0.0, 0.0])
        for i in (range(0,3)):                        # calculating MOI, at nacelle center of gravity with origin at tower top center / yaw mass center, ignoring masses of non-drivetrain components / auxiliary systems
            # calculate moments around CM
            # sum moments around each components CM (adjust for mass of mainframe) # TODO: add yaw MMI
            I[i]  =  self.lowSpeedShaftI[i] + self.mainBearingI[i] + self.secondBearingI[i] + self.gearboxI[i] + \
                          self.highSpeedSideI[i] + self.generatorI[i] + self.bedplateI[i] * (self.mainframeMass / self.bedplateMass)
            # translate to nacelle CM using parallel axis theorem (use mass of mainframe en lieu of bedplate to account for auxiliary equipment)
            for j in (range(0,3)): 
                if i != j:
                    I[i] +=  self.lowSpeedShaftMass * (self.lowSpeedShaftCM[i] - cm[i]) ** 2 + \
                                  self.mainBearingMass * (self.mainBearingCM[i] - cm[i]) ** 2 + \
                                  self.secondBearingMass * (self.secondBearingCM[i] - cm[i]) ** 2 + \
                                  self.gearboxMass * (self.gearboxCM[i] - cm[i]) ** 2 + \
                                  self.highSpeedSideMass * (self.highSpeedSideCM[i] - cm[i]) ** 2 + \
                                  self.generatorMass * (self.generatorCM[i] - cm[i]) ** 2 + \
                                  self.mainframeMass * (self.bedplateCM[i] - cm[i]) ** 2
        self.I = I
        
        # derivatives
        d_mass_d_aboveYawMass = 1.0
        d_mass_d_yawMass = 1.0
        
        d_cm_d_lowSpeedShaftMass = np.array([0.0, 0.0, 0.0])
        for i in (range(0,3)):
            d_cm_d_lowSpeedShaftMass[i] = (self.lowSpeedShaftCM[i]*self.mass - (self.lowSpeedShaftMass * self.lowSpeedShaftCM[i] + self.mainBearingMass * self.mainBearingCM[i] + \
                    self.secondBearingMass * self.secondBearingCM[i] + self.gearboxMass * self.gearboxCM[i] + self.highSpeedSideMass * self.highSpeedSideCM[i] + \
                    self.generatorMass * self.generatorCM[i] + self.mainframeMass * self.bedplateCM[i]))/ ((self.mass)**2)
        d_cm_d_mainBearingMass = np.array([0.0, 0.0, 0.0])
        for i in (range(0,3)):
            d_cm_d_mainBearingMass[i] = (self.mainBearingCM[i]*self.mass - (self.lowSpeedShaftMass * self.lowSpeedShaftCM[i] + self.mainBearingMass * self.mainBearingCM[i] + \
                    self.secondBearingMass * self.secondBearingCM[i] + self.gearboxMass * self.gearboxCM[i] + self.highSpeedSideMass * self.highSpeedSideCM[i] + \
                    self.generatorMass * self.generatorCM[i] + self.mainframeMass * self.bedplateCM[i]))/ ((self.mass)**2)
        d_cm_d_secondBearingMass = np.array([0.0, 0.0, 0.0])
        for i in (range(0,3)):
            d_cm_d_secondBearingMass[i] = (self.secondBearingCM[i]*self.mass - (self.lowSpeedShaftMass * self.lowSpeedShaftCM[i] + self.mainBearingMass * self.mainBearingCM[i] + \
                    self.secondBearingMass * self.secondBearingCM[i] + self.gearboxMass * self.gearboxCM[i] + self.highSpeedSideMass * self.highSpeedSideCM[i] + \
                    self.generatorMass * self.generatorCM[i] + self.mainframeMass * self.bedplateCM[i]))/ ((self.mass)**2)
        d_cm_d_gearboxMass = np.array([0.0, 0.0, 0.0])
        for i in (range(0,3)):
            d_cm_d_gearboxMass[i] = (self.gearboxCM[i]*self.mass - (self.lowSpeedShaftMass * self.lowSpeedShaftCM[i] + self.mainBearingMass * self.mainBearingCM[i] + \
                    self.secondBearingMass * self.secondBearingCM[i] + self.gearboxMass * self.gearboxCM[i] + self.highSpeedSideMass * self.highSpeedSideCM[i] + \
                    self.generatorMass * self.generatorCM[i] + self.mainframeMass * self.bedplateCM[i]))/ ((self.mass)**2)
        d_cm_d_highSpeedSideMass = np.array([0.0, 0.0, 0.0])
        for i in (range(0,3)):
            d_cm_d_highSpeedSideMass[i] = (self.highSpeedSideCM[i]*self.mass - (self.lowSpeedShaftMass * self.lowSpeedShaftCM[i] + self.mainBearingMass * self.mainBearingCM[i] + \
                    self.secondBearingMass * self.secondBearingCM[i] + self.gearboxMass * self.gearboxCM[i] + self.highSpeedSideMass * self.highSpeedSideCM[i] + \
                    self.generatorMass * self.generatorCM[i] + self.mainframeMass * self.bedplateCM[i]))/ ((self.mass)**2)
        d_cm_d_generatorMass = np.array([0.0, 0.0, 0.0])
        for i in (range(0,3)):
            d_cm_d_generatorMass[i] = (self.generatorCM[i]*self.mass - (self.lowSpeedShaftMass * self.lowSpeedShaftCM[i] + self.mainBearingMass * self.mainBearingCM[i] + \
                    self.secondBearingMass * self.secondBearingCM[i] + self.gearboxMass * self.gearboxCM[i] + self.highSpeedSideMass * self.highSpeedSideCM[i] + \
                    self.generatorMass * self.generatorCM[i] + self.mainframeMass * self.bedplateCM[i]))/ ((self.mass)**2)
        d_cm_d_mainframeMass = np.array([0.0, 0.0, 0.0])
        for i in (range(0,3)):
            d_cm_d_mainframeMass[i] = (self.bedplateCM[i]*self.mass - (self.lowSpeedShaftMass * self.lowSpeedShaftCM[i] + self.mainBearingMass * self.mainBearingCM[i] + \
                    self.secondBearingMass * self.secondBearingCM[i] + self.gearboxMass * self.gearboxCM[i] + self.highSpeedSideMass * self.highSpeedSideCM[i] + \
                    self.generatorMass * self.generatorCM[i] + self.mainframeMass * self.bedplateCM[i]))/ ((self.mass)**2)

        d_cm_d_lowSpeedShaftCM = self.lowSpeedShaftMass / self.mass
        d_cm_d_mainBearingCM = self.mainBearingMass / self.mass
        d_cm_d_secondBearingCM = self.secondBearingMass / self.mass
        d_cm_d_gearboxCM = self.gearboxMass / self.mass
        d_cm_d_highSpeedSideCM = self.highSpeedSideMass / self.mass
        d_cm_d_generatorCM = self.generatorMass / self.mass
        d_cm_d_mainframeCM = self.mainframeMass / self.mass
        
        d_I_d_lowSpeedShaftI = 1
        d_I_d_mainBearingI = 1
        d_I_d_secondBearingI = 1
        d_I_d_gearboxI = 1
        d_I_d_highSpeedSideI = 1
        d_I_d_generatorI = 1
        d_I_d_mainframeI = 1
        
        d_I_d_lowSpeedShaftMass = np.array([0.0, 0.0, 0.0])
        for i in (range(0,3)):
            d_I_d_lowSpeedShaftMass[i] = (self.lowSpeedShaftCM[i]-self.cm[i])**2 + 2 * self.lowSpeedShaftMass * (self.lowSpeedShaftCM[i] - self.cm[i]) * d_cm_d_lowSpeedShaftCM        
        d_I_d_mainBearingMass = np.array([0.0, 0.0, 0.0])
        for i in (range(0,3)):
            d_I_d_mainBearingMass[i] = (self.mainBearingCM[i]-self.cm[i])**2 + 2 * self.mainBearingMass * (self.mainBearingCM[i] - self.cm[i]) * d_cm_d_mainBearingCM  
        d_I_d_secondBearingMass = np.array([0.0, 0.0, 0.0])
        for i in (range(0,3)):
            d_I_d_secondBearingMass[i] = (self.secondBearingCM[i]-self.cm[i])**2 + 2 * self.secondBearingMass * (self.secondBearingCM[i] - self.cm[i]) * d_cm_d_secondBearingCM
        d_I_d_gearboxMass = np.array([0.0, 0.0, 0.0])
        for i in (range(0,3)):
            d_I_d_gearboxMass[i] = (self.gearboxCM[i]-self.cm[i])**2 + 2 * self.gearboxMass * (self.gearboxCM[i] - self.cm[i]) * d_cm_d_gearboxCM        
        d_I_d_highSpeedSideMass = np.array([0.0, 0.0, 0.0])
        for i in (range(0,3)):
            d_I_d_highSpeedSideMass[i] = (self.highSpeedSideCM[i]-self.cm[i])**2 + 2 * self.highSpeedSideMass * (self.highSpeedSideCM[i] - self.cm[i]) * d_cm_d_highSpeedSideCM  
        d_I_d_generatorMass = np.array([0.0, 0.0, 0.0])
        for i in (range(0,3)):
            d_I_d_generatorMass[i] = (self.generatorCM[i]-self.cm[i])**2 + 2 * self.generatorMass * (self.generatorCM[i] - self.cm[i]) * d_cm_d_generatorCM
        d_I_d_mainframeMass = np.array([0.0, 0.0, 0.0])
        for i in (range(0,3)):
            d_I_d_mainframeMass[i] = (self.bedplateCM[i]-self.cm[i])**2 + 2 * self.mainframeMass * (self.bedplateCM[i] - self.cm[i]) * d_cm_d_mainframeCM

        d_I_d_lowSpeedShaftCM = np.array([0.0, 0.0, 0.0])
        for i in range(0,3):
        	  d_I_d_lowSpeedShaftCM[i] = 2 * self.lowSpeedShaftMass * (self.lowSpeedShaftCM[i] - self.cm[i]) * (1 - d_cm_d_lowSpeedShaftCM)
        d_I_d_mainBearingCM = np.array([0.0, 0.0, 0.0])
        for i in range(0,3):
        	  d_I_d_mainBearingCM[i] = 2 * self.mainBearingMass * (self.mainBearingCM[i] - self.cm[i]) * (1 - d_cm_d_mainBearingCM)
        d_I_d_secondBearingCM = np.array([0.0, 0.0, 0.0])
        for i in range(0,3):
        	  d_I_d_secondBearingCM[i] = 2 * self.secondBearingMass * (self.secondBearingCM[i] - self.cm[i]) * (1 - d_cm_d_secondBearingCM)
        d_I_d_gearboxCM = np.array([0.0, 0.0, 0.0])
        for i in range(0,3):
        	  d_I_d_gearboxCM[i] = 2 * self.gearboxMass * (self.gearboxCM[i] - self.cm[i]) * (1 - d_cm_d_gearboxCM)
        d_I_d_highSpeedSideCM = np.array([0.0, 0.0, 0.0])
        for i in range(0,3):
        	  d_I_d_highSpeedSideCM[i] = 2 * self.highSpeedSideMass * (self.highSpeedSideCM[i] - self.cm[i]) * (1 - d_cm_d_highSpeedSideCM)
        d_I_d_generatorCM = np.array([0.0, 0.0, 0.0])
        for i in range(0,3):
        	  d_I_d_generatorCM[i] = 2 * self.generatorMass * (self.generatorCM[i] - self.cm[i]) * (1 - d_cm_d_generatorCM)
        d_I_d_mainframeCM = np.array([0.0, 0.0, 0.0])
        for i in range(0,3):
        	  d_I_d_mainframeCM[i] = 2 * self.mainframeMass * (self.bedplateCM[i] - self.cm[i]) * (1 - d_cm_d_mainframeCM)

        # Jacobian
        self.J = np.array([[d_mass_d_aboveYawMass, d_mass_d_yawMass, 0, 0, 0, 0, 0, 0, 0, \
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, \
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], \
                           [0, 0, d_cm_d_lowSpeedShaftMass[0], d_cm_d_mainBearingMass[0], d_cm_d_secondBearingMass[0], d_cm_d_gearboxMass[0], d_cm_d_highSpeedSideMass[0], d_cm_d_generatorMass[0], d_cm_d_mainframeMass[0], \
                            d_cm_d_lowSpeedShaftCM, 0, 0, d_cm_d_mainBearingCM, 0, 0, d_cm_d_secondBearingCM, 0, 0, d_cm_d_gearboxCM, 0, 0, d_cm_d_highSpeedSideCM, 0, 0, d_cm_d_generatorCM, 0, 0, d_cm_d_mainframeCM, 0, 0, \
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], \
                           [0, 0, d_cm_d_lowSpeedShaftMass[1], d_cm_d_mainBearingMass[1], d_cm_d_secondBearingMass[1], d_cm_d_gearboxMass[1], d_cm_d_highSpeedSideMass[1], d_cm_d_generatorMass[1], d_cm_d_mainframeMass[1], \
                            d_cm_d_lowSpeedShaftCM, 0, 0, d_cm_d_mainBearingCM, 0, 0, d_cm_d_secondBearingCM, 0, 0, d_cm_d_gearboxCM, 0, 0, d_cm_d_highSpeedSideCM, 0, 0, d_cm_d_generatorCM, 0, 0, d_cm_d_mainframeCM, 0, 0, \
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], \
                           [0, 0, d_cm_d_lowSpeedShaftMass[2], d_cm_d_mainBearingMass[2], d_cm_d_secondBearingMass[2], d_cm_d_gearboxMass[2], d_cm_d_highSpeedSideMass[2], d_cm_d_generatorMass[2], d_cm_d_mainframeMass[2], \
                            d_cm_d_lowSpeedShaftCM, 0, 0, d_cm_d_mainBearingCM, 0, 0, d_cm_d_secondBearingCM, 0, 0, d_cm_d_gearboxCM, 0, 0, d_cm_d_highSpeedSideCM, 0, 0, d_cm_d_generatorCM, 0, 0, d_cm_d_mainframeCM, 0, 0, \
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], \
                           [0, 0, d_I_d_lowSpeedShaftMass[0], d_I_d_mainBearingMass[0], d_I_d_secondBearingMass[0], d_I_d_gearboxMass[0], d_I_d_highSpeedSideMass[0], d_I_d_generatorMass[0], d_I_d_mainframeMass[0], \
                           d_I_d_lowSpeedShaftCM[0], 0, 0, d_I_d_mainBearingCM[0], 0, 0, d_I_d_secondBearingCM[0], 0, 0, d_I_d_gearboxCM[0], 0, 0, d_I_d_highSpeedSideCM[0], 0, 0, d_I_d_generatorCM[0], 0, 0, d_I_d_mainframeCM[0], 0, 0, \
                           d_I_d_lowSpeedShaftI, 0, 0, d_I_d_mainBearingI, 0, 0, d_I_d_secondBearingI, 0, 0, d_I_d_gearboxI, 0, 0, d_I_d_highSpeedSideI, 0, 0, d_I_d_generatorI, 0, 0, d_I_d_mainframeI, 0, 0], \
                           [0, 0, d_I_d_lowSpeedShaftMass[1], d_I_d_mainBearingMass[1], d_I_d_secondBearingMass[1], d_I_d_gearboxMass[1], d_I_d_highSpeedSideMass[1], d_I_d_generatorMass[1], d_I_d_mainframeMass[1], \
                           d_I_d_lowSpeedShaftCM[1], 0, 0, d_I_d_mainBearingCM[1], 0, 0, d_I_d_secondBearingCM[1], 0, 0, d_I_d_gearboxCM[1], 0, 0, d_I_d_highSpeedSideCM[1], 0, 0, d_I_d_generatorCM[1], 0, 0, d_I_d_mainframeCM[1], 0, 0, \
                           d_I_d_lowSpeedShaftI, 0, 0, d_I_d_mainBearingI, 0, 0, d_I_d_secondBearingI, 0, 0, d_I_d_gearboxI, 0, 0, d_I_d_highSpeedSideI, 0, 0, d_I_d_generatorI, 0, 0, d_I_d_mainframeI, 0, 0], \
                           [0, 0, d_I_d_lowSpeedShaftMass[2], d_I_d_mainBearingMass[2], d_I_d_secondBearingMass[2], d_I_d_gearboxMass[2], d_I_d_highSpeedSideMass[2], d_I_d_generatorMass[2], d_I_d_mainframeMass[2], \
                           d_I_d_lowSpeedShaftCM[2], 0, 0, d_I_d_mainBearingCM[2], 0, 0, d_I_d_secondBearingCM[2], 0, 0, d_I_d_gearboxCM[2], 0, 0, d_I_d_highSpeedSideCM[2], 0, 0, d_I_d_generatorCM[2], 0, 0, d_I_d_mainframeCM[2], 0, 0, \
                           d_I_d_lowSpeedShaftI, 0, 0, d_I_d_mainBearingI, 0, 0, d_I_d_secondBearingI, 0, 0, d_I_d_gearboxI, 0, 0, d_I_d_highSpeedSideI, 0, 0, d_I_d_generatorI, 0, 0, d_I_d_mainframeI, 0, 0]])

    def provideJ(self):

        input_keys = ['aboveYawMass', 'yawMass', 'lowSpeedShaftMass', 'mainBearingMass', 'secondBearingMass', 'gearboxMass', 'highSpeedSideMass', 'generatorMass', 'mainframeMass', \
                      'lowSpeedShaftCM[0]', 'mainBearingCM[0]', 'secondBearingCM[0]', 'gearboxCM[0]', 'highSpeedSideCM[0]', 'generatorCM[0]', 'bedplateCM[0]', \
                      'lowSpeedShaftCM[1]', 'mainBearingCM[1]', 'secondBearingCM[1]', 'gearboxCM[1]', 'highSpeedSideCM[1]', 'generatorCM[1]', 'bedplateCM[1]', \
                      'lowSpeedShaftCM[2]', 'mainBearingCM[2]', 'secondBearingCM[2]', 'gearboxCM[2]', 'highSpeedSideCM[2]', 'generatorCM[2]', 'bedplateCM[2]', \
                      'lowSpeedShaftI[0]', 'mainBearingI[0]', 'secondBearingI[0]', 'gearboxI[0]', 'highSpeedSideI[0]', 'generatorI[0]', 'bedplateI[0]', \
                      'lowSpeedShaftI[1]', 'mainBearingI[1]', 'secondBearingI[1]', 'gearboxI[1]', 'highSpeedSideI[1]', 'generatorI[1]', 'bedplateI[1]', \
                      'lowSpeedShaftI[2]', 'mainBearingI[2]', 'secondBearingI[2]', 'gearboxI[2]', 'highSpeedSideI[2]', 'generatorI[2]', 'bedplateI[2]']
        output_keys = ['mass', 'cm[0]', 'cm[1]', 'cm[2]', 'I[0]', 'I[1]', 'I[2]']

        self.derivatives.set_first_derivative(input_keys, output_keys, self.J)

#--------------------------------------------------------------------------------------------------------

if __name__ == '__main__':
     pass