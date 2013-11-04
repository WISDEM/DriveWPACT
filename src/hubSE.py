"""
hubSE.py

Created by Katherine Dykes 2012.
Copyright (c) NREL. All rights reserved.
"""

from openmdao.main.api import Component, Assembly
from openmdao.main.datatypes.api import Array, Float, Bool, Int
import numpy as np
from math import pi

#-------------------------------------------------------------------------------

class Hub(Component):
    ''' Hub class    
          The Hub class is used to represent the hub component of a wind turbine. 
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.            
    '''

    # variables
    rootMoment = Float(iotype='in', units='N*m', desc='flapwise bending moment at blade root')
    rotorDiameter = Float(iotype='in', units='m', desc='rotor diameter')
    
    # parameters
    hubDiameter = Float(0.0, iotype='in', units='m', desc='hub diameter')
    bladeNumber = Int(3, iotype='in', desc='number of turbine blades')

    # outputs
    mass = Float(0.0, iotype='out', units='kg', desc='overall component mass')
    cm = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system')
    I = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass')
    
    def __init__(self):
        ''' 
        Initializes hub component 
        
        Parameters
        ----------
        rootMoment : float
          maximum flap-wise root moment for the blade (if == 0, then it is set within compute method) [Nm]
        rotorDiameter : float
          The wind turbine rotor diameter [m]
        hubDiameter : float
          the specified hub diameter (if == 0, then it is set within compute method) [m]
        bladeNumber : int
          Number of wind turbine rotor blades
        
        Returns
        -------
        mass : float
          mass of the component [kg]
        cm : array of float
          center of mass of the component (relative to tower top center) [m, m, m]
        I : array of float
          principle moments of inertia for the component (Ixx, iyy, Izz) around its center of mass (relative to tower top center) [kg*m^2, kg*m^2, kg*m^2]   
        '''

        super(Hub, self).__init__()

    def execute(self):

        # Sunderland method for calculating hub, pitch and spinner cone masses
        hubloadFact      = 1                                     # default is 3 blade rigid hub (should depend on hub type)
        hubgeomFact      = 1                                     # default is 3 blade (should depend on blade number)
        hubcontFact      = 1.5                                   # default is 1.5 for full span pitch control (should depend on control method)
        hubmatldensity   = 7860.0                               # density of hub material (kg / m^3) - assuming BS1503-622 (same material as LSS)
        hubmatlstress    = 371000000.0                                # allowable stress of hub material (N / m^2)

        # Root moment required as input, could be undone
        '''if RootMoment == 0.0:
            RootMoment = (3.06 * pi / 8) * AirDensity * (RatedWindSpeed ** 2) * (Solidity * (RotorDiam ** 3)) / BladeNum
                                                            # simplified equation for blade root moment (Sunderland model) if one is not provided'''
        
        self.mass =50 * hubgeomFact * hubloadFact * hubcontFact * self.bladeNumber * self.rootMoment * (hubmatldensity / hubmatlstress)
                                                            # mass of hub based on Sunderland model
                                                            # 31.4 adapted to 50 to fit 5 MW data

        # calculate mass properties
        if self.hubDiameter == 0:
            self.diameter =(3.30)
        else:
            self.diameter =(self.hubDiameter)
        self.thickness = self.diameter * (0.055 / 3.30)         # 0.055 for 1.5 MW outer diameter of 3.3 - using proportional constant

        cm = np.array([0.0,0.0,0.0])
        cm[0]     = - (0.05 * self.rotorDiameter)
        cm[1]     = 0.0
        cm[2]     = 0.025 * self.rotorDiameter
        self.cm = (cm)

        I = np.array([0.0, 0.0, 0.0])
        I[0] = 0.4 * (self.mass) * ((self.diameter / 2) ** 5 - (self.diameter / 2 - self.thickness) ** 5) / \
               ((self.diameter / 2) ** 3 - (self.diameter / 2 - self.thickness) ** 3)
        I[1] = I[0]
        I[2] = I[1]
        self.I = (I)

        # derivatives
        d_hubMass_d_rootMoment = 50 * hubgeomFact * hubloadFact * hubcontFact * self.bladeNumber * (hubmatldensity / hubmatlstress)
        d_cm_d_rotorDiameter = np.array([-0.05, 0.0, 0.025])
        d_I_d_rootMoment = 0.4 * d_hubMass_d_rootMoment * ((0.5**5 - (0.05 - 0.055/3.3)**5)/(0.5**3 - (0.05 - 0.055/3.3)**3)) * self.diameter**2
        d_I_d_hubDiameter = 2 * 0.4 * self.mass * ((0.5**5 - (0.05 - 0.055/3.3)**5)/(0.5**3 - (0.05 - 0.055/3.3)**3)) * self.diameter
        
        # Jacobian
        self.J = np.array([[d_hubMass_d_rootMoment, 0, 0], \
                           [0, d_cm_d_rotorDiameter[0], 0], \
                           [0, d_cm_d_rotorDiameter[1], 0], \
                           [0, d_cm_d_rotorDiameter[2], 0], \
                           [d_I_d_rootMoment, 0, d_I_d_hubDiameter], \
                           [d_I_d_rootMoment, 0, d_I_d_hubDiameter], \
                           [d_I_d_rootMoment, 0, d_I_d_hubDiameter]])

    def provideJ(self):

        input_keys = ['rootMoment', 'rotorDiameter', 'hubDiameter']
        output_keys = ['hubMass', 'cm[0]', 'cm[1]', 'cm[2]', 'I[0]', 'I[1]', 'I[2]']

        self.derivatives.set_first_derivative(input_keys, output_keys, self.J)

#-------------------------------------------------------------------------------

class PitchSystem(Component): 
    '''
     PitchSystem class          
      The PitchSystem class is used to represent the pitch system of a wind turbine. 
      It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
      It contains an update method to determine the mass, mass properties, and dimensions of the component.
    '''

    # variables
    bladeMass = Float(iotype='in', units='kg', desc='mass of one blade')
    rootMoment = Float(iotype='in', units='N*m', desc='flapwise bending moment at blade root')
    rotorDiameter = Float(iotype='in', units='m', desc='rotor diameter')
    
    # parameters
    hubDiameter = Float(0.0, iotype='in', units='m', desc='hub diameter')
    bladeNumber = Int(3, iotype='in', desc='number of turbine blades')

    # outputs
    mass = Float(0.0, iotype='out', units='kg', desc='overall component mass')
    cm = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system')
    I = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass')
    
    def __init__(self):
        ''' 
        Initializes pitch system
        
        Parameters
        ----------
        bladeMass : float
          The wind turbine individual blade mass [kg]
        rootMoment : float
          maximum flap-wise root moment for the blade (if == 0, then it is set within compute method) [Nm]
        rotorDiameter : float
          The wind turbine rotor diameter [m]
        hubDiameter : float
          the specified hub diameter (if == 0, then it is set within compute method) [m]
        bladeNumber : int
          Number of wind turbine rotor blades

        Returns
        -------
        mass : float
          mass of the component [kg]
        cm : array of float
          center of mass of the component (relative to tower top center) [m, m, m]
        I : array of float
          principle moments of inertia for the component (Ixx, iyy, Izz) around its center of mass (relative to tower top center) [kg*m^2, kg*m^2, kg*m^2]
        '''

        super(PitchSystem, self).__init__()

    def execute(self):

        # Sunderland method for calculating pitch system masses
        pitchmatldensity = 7860.0                             # density of pitch system material (kg / m^3) - assuming BS1503-622 (same material as LSS)
        pitchmatlstress  = 371000000.0                              # allowable stress of hub material (N / m^2)

        # Root moment required as input, could be undone
        '''if RootMoment == 0.0:
            RootMoment = (3.06 * pi / 8) * AirDensity * (RatedWindSpeed ** 2) * (Solidity * (RotorDiam ** 3)) / BladeNum
                                                            # simplified equation for blade root moment (Sunderland model) if one is not provided'''

        hubpitchFact      = 1.0                                 # default factor is 1.0 (0.54 for modern designs)
        self.mass =hubpitchFact * (0.22 * self.bladeMass * self.bladeNumber + 12.6 * self.bladeNumber * self.rootMoment * (pitchmatldensity / pitchmatlstress))
                                                            # mass of pitch system based on Sunderland model

        # calculate mass properties
        if self.hubDiameter == 0:
            self.diameter =(3.30)
        else:
            self.diameter =(self.hubDiameter)

        # calculate mass properties
        cm = np.array([0.0,0.0,0.0])
        cm[0]     = - (0.05 * self.rotorDiameter)
        cm[1]     = 0.0
        cm[2]     = 0.025 * self.rotorDiameter
        self.cm = (cm)

        I = np.array([0.0, 0.0, 0.0])
        I[0] = self.mass * (self.diameter ** 2) / 4
        I[1] = I[0]
        I[2] = I[1]
        self.I = (I)

        # derivatives
        d_pitchMass_d_bladeMass = hubpitchFact * 0.22 * self.bladeNumber
        d_pitchMass_d_rootMoment = hubpitchFact * 12.6 * self.bladeNumber * pitchmatldensity / pitchmatlstress
        d_cm_d_rotorDiameter = np.array([-0.05, 0.0, 0.025])
        d_I_d_bladeMass = d_pitchMass_d_bladeMass * (self.diameter**2) / 4
        d_I_d_rootMoment = d_pitchMass_d_rootMoment * (self.diameter**2) / 4
        d_I_d_hubDiameter = self.mass * (2/4) * self.diameter

        # Jacobian
        self.J = np.array([[d_pitchMass_d_bladeMass, d_pitchMass_d_rootMoment, 0, 0], \
                           [0, 0, d_cm_d_rotorDiameter[0], 0], \
                           [0, 0, d_cm_d_rotorDiameter[1], 0], \
                           [0, 0, d_cm_d_rotorDiameter[2], 0], \
                           [d_I_d_bladeMass, d_I_d_rootMoment, 0, d_I_d_hubDiameter], \
                           [d_I_d_bladeMass, d_I_d_rootMoment, 0, d_I_d_hubDiameter], \
                           [d_I_d_bladeMass, d_I_d_rootMoment, 0, d_I_d_hubDiameter]])

    def provideJ(self):

        input_keys = ['bladeMass', 'rootMoment', 'rotorDiameter', 'hubDiameter']
        output_keys = ['pitchMass', 'cm[0]', 'cm[1]', 'cm[2]', 'I[0]', 'I[1]', 'I[2]']

        self.derivatives.set_first_derivative(input_keys, output_keys, self.J)

#-------------------------------------------------------------------------------      

class Spinner(Component): 
    ''' 
       Spinner class
          The SpinnerClass is used to represent the spinner of a wind turbine. 
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.
    '''
    
    # variables
    rotorDiameter = Float(iotype='in', units='m', desc='rotor diameter')

    # parameters
    hubDiameter = Float(0.0, iotype='in', units='m', desc='hub diameter')

    # outputs
    mass = Float(0.0, iotype='out', units='kg', desc='overall component mass')
    cm = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system')
    I = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass')
    
    def __init__(self):
        ''' 
        Initializes spinner system
        
        Parameters
        ----------
        rotorDiameter : float
          The wind turbine rotor diameter [m]
        hubDiameter : float
          the specified hub diameter (if == 0, then it is set within compute method) [m]

        Returns
        -------
        mass : float
          mass of the component [kg]
        cm : array of float
          center of mass of the component (relative to tower top center) [m, m, m]
        I : array of float
          principle moments of inertia for the component (Ixx, iyy, Izz) around its center of mass (relative to tower top center) [kg*m^2, kg*m^2, kg*m^2]
        '''

        super(Spinner, self).__init__()

    def execute(self):

        self.mass =18.5 * self.rotorDiameter + (-520.5)   # spinner mass comes from cost and scaling model

        # calculate mass properties
        if self.hubDiameter == 0:
            self.diameter =(3.30)
        else:
            self.diameter =(self.hubDiameter)
        self.thickness = self.diameter * (0.055 / 3.30)         # 0.055 for 1.5 MW outer diameter of 3.3 - using proportional constant

        cm = np.array([0.0,0.0,0.0])
        cm[0]     = - (0.05 * self.rotorDiameter)
        cm[1]     = 0.0
        cm[2]     = 0.025 * self.rotorDiameter
        self.cm = (cm)

        I = np.array([0.0, 0.0, 0.0])
        I[0] = 0.4 * (self.mass) * ((self.diameter / 2) ** 5 - (self.diameter / 2 - self.thickness) ** 5) / \
               ((self.diameter / 2) ** 3 - (self.diameter / 2 - self.thickness) ** 3)
        I[1] = I[0]
        I[2] = I[1]
        self.I = (I)

        # derivatives
        d_spinnerMass_d_rotorDiameter = 18.5
        d_cm_d_rotorDiameter = np.array([-0.05, 0.0, 0.025])
        d_I_d_rotorDiameter = 0.4 * d_spinnerMass_d_rotorDiameter * ((0.5**5 - (0.05 - 0.055/3.3)**5)/(0.5**3 - (0.05 - 0.055/3.3)**3)) * self.diameter**2
        d_I_d_hubDiameter = 2 * 0.4 * self.mass * ((0.5**5 - (0.05 - 0.055/3.3)**5)/(0.5**3 - (0.05 - 0.055/3.3)**3)) * self.diameter

        # Jacobian
        self.J = np.array([[d_spinnerMass_d_rotorDiameter, 0], \
                           [d_cm_d_rotorDiameter[0], 0], \
                           [d_cm_d_rotorDiameter[1], 0], \
                           [d_cm_d_rotorDiameter[2], 0], \
                           [d_I_d_rotorDiameter, d_I_d_hubDiameter], \
                           [d_I_d_rotorDiameter, d_I_d_hubDiameter], \
                           [d_I_d_rotorDiameter, d_I_d_hubDiameter]])

    def provideJ(self):

        input_keys = ['rotorDiameter', 'hubDiameter']
        output_keys = ['spinnerMass', 'cm[0]', 'cm[1]', 'cm[2]', 'I[0]', 'I[1]', 'I[2]']

        self.derivatives.set_first_derivative(input_keys, output_keys, self.J)

#-------------------------------------------------------------------------------

class HubSystemAdder(Component): 
    ''' 
       HubSystem class
          The HubSystem class is used to represent the hub system of a wind turbine. 
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.            
    '''
    
    # variables    
    hubMass = Float(iotype='in', units='kg', desc='overall component mass')
    hubCM = Array(np.array([0.0, 0.0, 0.0]), iotype='in', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system')
    hubI = Array(np.array([0.0, 0.0, 0.0]), iotype='in', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass')
    pitchMass = Float(iotype='in', units='kg', desc='overall component mass')
    pitchCM = Array(np.array([0.0, 0.0, 0.0]), iotype='in', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system')
    pitchI = Array(np.array([0.0, 0.0, 0.0]), iotype='in', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass')
    spinnerMass = Float(iotype='in', units='kg', desc='overall component mass')
    spinnerCM = Array(np.array([0.0, 0.0, 0.0]), iotype='in', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system')
    spinnerI = Array(np.array([0.0, 0.0, 0.0]), iotype='in', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass')

    # outputs
    mass = Float(0.0, iotype='out', units='kg', desc='overall component mass')
    cm = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system')
    I = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass')
    
    
    def __init__(self):
        ''' 
        Initializes hub system component 
        
        Parameters
        ----------
        hubMass : float
          mass of component [kg]
        hubCM : array of float
          center of mass of component [m, m, m]
        hubI : array of float
          mass moments of inertia for compnent [kg*m^2, kg*m^2, kg*m^2]
        pitchMass : float
          mass of component [kg]
        pitchCM : array of float
          center of mass of component [m, m, m]
        pitchI : array of float
          mass moments of inertia for compnent [kg*m^2, kg*m^2, kg*m^2]
        spinnerMass : float
          mass of component [kg]
        spinnerCM : array of float
          center of mass of component [m, m, m]
        spinnerI : array of float
          mass moments of inertia for compnent [kg*m^2, kg*m^2, kg*m^2]

        Returns
        -------
        mass : float
          mass of the component [kg]
        cm : array of float
          center of mass of the component (relative to tower top center) [m, m, m]
        I : array of float
          moments of inertia for the component (Ixx, iyy, Izz) around its center of mass (relative to tower top center) [kg*m^2, kg*m^2, kg*m^2]
        '''

        super(HubSystemAdder, self).__init__()

    def execute(self):

        self.mass = self.hubMass + self.pitchMass + self.spinnerMass

        # calculate mass properties
        # calculation of mass center and moments of inertia
        cm = np.array([0.0,0.0,0.0])
        for i in (range(0,3)):
            # calculate center of mass
            cm[i] = (self.hubMass * self.hubCM[i] + self.pitchMass * self.pitchCM[i] + \
                    self.spinnerMass * self.spinnerCM[i] ) / (self.mass)
        self.cm = cm

        I = np.array([0.0, 0.0, 0.0])
        for i in (range(0,3)):                        # calculating MOI, at nacelle center of gravity with origin at tower top center / yaw mass center, ignoring masses of non-drivetrain components / auxiliary systems
            # calculate moments around CM
            # sum moments around each components CM
            I[i]  =  self.hubI[i] + self.pitchI[i] + self.spinnerI[i]
            # translate to hub system CM using parallel axis theorem
            for j in (range(0,3)): 
                if i != j:
                    I[i] +=  (self.hubMass * (self.hubCM[i] - self.cm[i]) ** 2) + \
                                  (self.pitchMass * (self.pitchCM[i] - self.cm[i]) ** 2) + \
                                  (self.spinnerMass * (self.spinnerCM[i] - self.cm[i]) ** 2)
        self.I = I

        # derivatives
        d_mass_d_hubMass = 1
        d_mass_d_pitchMass = 1
        d_mass_d_spinnerMass = 1

        d_cm_d_hubMass = np.array([0.0, 0.0, 0.0])
        for i in (range(0,3)):
            d_cm_d_hubMass[i] = (self.hubCM[i]*self.mass - (self.hubMass * self.hubCM[i] + self.pitchMass * self.pitchCM[i] + \
                    self.spinnerMass * self.spinnerCM[i] ))/ ((self.mass)**2)
        d_cm_d_pitchMass = np.array([0.0, 0.0, 0.0])
        for i in (range(0,3)):
            d_cm_d_pitchMass[i] = (self.pitchCM[i]*self.mass - (self.hubMass * self.hubCM[i] + self.pitchMass * self.pitchCM[i] + \
                    self.spinnerMass * self.spinnerCM[i] ))/ ((self.mass)**2)
        d_cm_d_spinnerMass = np.array([0.0, 0.0, 0.0])
        for i in (range(0,3)):
            d_cm_d_spinnerMass[i] = (self.spinnerCM[i]*self.mass - (self.hubMass * self.hubCM[i] + self.pitchMass * self.pitchCM[i] + \
                    self.spinnerMass * self.spinnerCM[i] ))/ ((self.mass)**2)

        d_cm_d_hubCM = self.hubMass / self.mass
        d_cm_d_pitchCM = self.pitchMass / self.mass
        d_cm_d_spinnerCM = self.spinnerMass / self.mass

        d_I_d_hubI = 1
        d_I_d_pitchI = 1
        d_I_d_spinnerI = 1
        
        d_I_d_hubMass = np.array([0.0, 0.0, 0.0])
        for i in (range(0,3)):
            d_I_d_hubMass[i] = (self.hubCM[i]-self.cm[i])**2 + 2 * self.hubMass * (self.hubCM[i] - self.cm[i]) * d_cm_d_hubCM        
        d_I_d_pitchMass = np.array([0.0, 0.0, 0.0])
        for i in (range(0,3)):
            d_I_d_pitchMass[i] = (self.pitchCM[i]-self.cm[i])**2 + 2 * self.pitchMass * (self.pitchCM[i] - self.cm[i]) * d_cm_d_pitchCM  
        d_I_d_spinnerMass = np.array([0.0, 0.0, 0.0])
        for i in (range(0,3)):
            d_I_d_spinnerMass[i] = (self.spinnerCM[i]-self.cm[i])**2 + 2 * self.spinnerMass * (self.spinnerCM[i] - self.cm[i]) * d_cm_d_spinnerCM

        d_I_d_hubCM = np.array([0.0, 0.0, 0.0])
        for i in range(0,3):
        	  d_I_d_hubCM[i] = 2 * self.hubMass * (self.hubCM[i] - self.cm[i]) * (1 - d_cm_d_hubCM)
        d_I_d_pitchCM = np.array([0.0, 0.0, 0.0])
        for i in range(0,3):
        	  d_I_d_pitchCM[i] = 2 * self.hubMass * (self.pitchCM[i] - self.cm[i]) * (1 - d_cm_d_pitchCM)
        d_I_d_spinnerCM = np.array([0.0, 0.0, 0.0])
        for i in range(0,3):
        	  d_I_d_spinnerCM[i] = 2 * self.spinnerMass * (self.spinnerCM[i] - self.cm[i]) * (1 - d_cm_d_spinnerCM)

        # Jacobian
        self.J = np.array([[d_mass_d_hubMass, d_mass_d_pitchMass, d_mass_d_spinnerMass, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], \
                           [d_cm_d_hubMass[0], d_cm_d_pitchMass[0], d_cm_d_spinnerMass[0], d_cm_d_hubCM, 0, 0, d_cm_d_pitchCM, 0, 0, d_cm_d_spinnerCM, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], \
                           [d_cm_d_hubMass[1], d_cm_d_pitchMass[1], d_cm_d_spinnerMass[1], 0, d_cm_d_hubCM, 0, 0, d_cm_d_pitchCM, 0, 0, d_cm_d_spinnerCM, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], \
                           [d_cm_d_hubMass[2], d_cm_d_pitchMass[2], d_cm_d_spinnerMass[2], 0, 0, d_cm_d_hubCM, 0, 0, d_cm_d_pitchCM, 0, 0, d_cm_d_spinnerCM, 0, 0, 0, 0, 0, 0, 0, 0, 0], \
                           [d_I_d_hubMass[0], d_I_d_pitchMass[0], d_I_d_spinnerMass[0], d_I_d_hubCM[0], 0, 0, d_I_d_pitchCM[0], 0, 0, d_I_d_spinnerCM[0], 0, 0, d_I_d_hubI, 0, 0, d_I_d_pitchI, 0, 0, d_I_d_spinnerI, 0, 0], \
                           [d_I_d_hubMass[1], d_I_d_pitchMass[1], d_I_d_spinnerMass[1], 0, d_I_d_hubCM[1], 0, 0, d_I_d_pitchCM[1], 0, 0, d_I_d_spinnerCM[1], 0, 0, d_I_d_hubI, 0, 0, d_I_d_pitchI, 0, 0, d_I_d_spinnerI, 0], \
                           [d_I_d_hubMass[2], d_I_d_pitchMass[2], d_I_d_spinnerMass[2], 0, 0, d_I_d_hubCM[2], 0, 0, d_I_d_pitchCM[2], 0, 0, d_I_d_spinnerCM[2], 0, 0, d_I_d_hubI, 0, 0, d_I_d_pitchI, 0, 0, d_I_d_spinnerI]])

    def provideJ(self):

        input_keys = ['hubMass', 'pitchMass', 'spinerMass', 'hubCM[0]', 'hubCM[1]', 'hubCM[2]', 'pitchCM[0]', 'pitchCM[1]', 'pitchCM[2]', \
                      'spinnerCM[0]', 'spinnerCM[1]', 'spinnerCM[2]',  'hubI[0]', 'hubI[1]', 'hubI[2]', 'pitchI[0]', 'pitchI[1]', 'pitchI[2]', \
                      'spinnerI[0]', 'spinnerI[1]', 'spinnerI[2]']
        output_keys = ['mass', 'cm[0]', 'cm[1]', 'cm[2]', 'I[0]', 'I[1]', 'I[2]']

        self.derivatives.set_first_derivative(input_keys, output_keys, self.J)

class HubSE(Assembly):
    ''' 
       HubSE class
          The HubSE class is used to represent the hub system of a wind turbine.             
    '''

    # variables
    bladeMass = Float(iotype='in', units='kg', desc='mass of one blade')
    rootMoment = Float(iotype='in', units='N*m', desc='flapwise bending moment at blade root')
    rotorDiameter = Float(iotype='in', units='m', desc='rotor diameter')
    
    # parameters
    hubDiameter = Float(0.0, iotype='in', units='m', desc='hub diameter')
    bladeNumber = Int(3, iotype='in', desc='number of turbine blades')

    def configure(self):

        # select components
        self.add('hubSystem', HubSystemAdder())
        self.add('hub', Hub())
        self.add('pitchSystem', PitchSystem())
        self.add('spinner', Spinner())
        
        # workflow
        self.driver.workflow.add(['hubSystem','hub', 'pitchSystem', 'spinner'])
        
        # connect inputs
        self.connect('bladeMass', ['pitchSystem.bladeMass'])
        self.connect('rootMoment', ['hub.rootMoment', 'pitchSystem.rootMoment'])
        self.connect('bladeNumber', ['hub.bladeNumber', 'pitchSystem.bladeNumber'])
        self.connect('rotorDiameter', ['hub.rotorDiameter', 'pitchSystem.rotorDiameter', 'spinner.rotorDiameter'])
        self.connect('hubDiameter', ['hub.hubDiameter', 'pitchSystem.hubDiameter', 'spinner.hubDiameter'])
        
        # connect components
        self.connect('hub.mass', 'hubSystem.hubMass')
        self.connect('hub.cm', 'hubSystem.hubCM')
        self.connect('hub.I', 'hubSystem.hubI')
        self.connect('pitchSystem.mass', 'hubSystem.pitchMass')
        self.connect('pitchSystem.cm', 'hubSystem.pitchCM')
        self.connect('pitchSystem.I', 'hubSystem.pitchI')
        self.connect('spinner.mass', 'hubSystem.spinnerMass')
        self.connect('spinner.cm', 'hubSystem.spinnerCM')
        self.connect('spinner.I', 'hubSystem.spinnerI')
        
        # create passthroughs
        self.create_passthrough('hubSystem.mass')
        self.create_passthrough('hubSystem.cm')
        self.create_passthrough('hubSystem.I')


#-------------------------------------------------------------------------------

def example():
  
    # simple test of module

    # NREL 5 MW turbine
    print "NREL 5 MW turbine test"
    hub = HubSE()
    hub.bladeMass = 17740.0 # kg
    hub.rotorDiameter = 126.0 # m
    hub.bladeNumber  = 3
    hub.hubDiameter   = 3.0 # m
    AirDensity= 1.225 # kg/(m^3)
    Solidity  = 0.0517 
    RatedWindSpeed = 11.05 # m/s
    hub.rootMoment = (3.06 * pi / 8) * AirDensity * (RatedWindSpeed ** 2) * (Solidity * (hub.rotorDiameter ** 3)) / hub.bladeNumber
    
    hub.run()

    print "Hub Components"
    print '  hub         {0:8.1f} kg'.format(hub.hub.mass)  # 31644.47
    print '  pitch mech  {0:8.1f} kg'.format(hub.pitchSystem.mass) # 17003.98
    print '  nose cone   {0:8.1f} kg'.format(hub.spinner.mass) # 1810.50
    print 'Hub system total {0:8.1f} kg'.format(hub.mass) # 50458.95
    print '    cm {0:6.2f} {1:6.2f} {2:6.2f}'.format(hub.cm[0], hub.cm[1], hub.cm[2])
    print '    I {0:6.1f} {1:6.1f} {2:6.1f}'.format(hub.I[0], hub.I[1], hub.I[2])

def example2():

    # WindPACT 1.5 MW turbine
    print "WindPACT 1.5 MW turbine test"
    hub = HubSE()
    hub.bladeMass = 4470.0
    hub.rotorDiameter = 70.0
    hub.bladeNumer  = 3
    hub.hubDiameter   = 3.0
    AirDensity= 1.225
    Solidity  = 0.065
    RatedWindSpeed = 12.12
    hub.rootMoment = (3.06 * pi / 8) * AirDensity * (RatedWindSpeed ** 2) * (Solidity * (hub.rotorDiameter ** 3)) / hub.bladeNumber

    hub.run()

    print "Hub Components"
    print '  hub         {0:8.1f} kg'.format(hub.hub.mass)
    print '  pitch mech  {0:8.1f} kg'.format(hub.pitchSystem.mass)
    print '  nose cone   {0:8.1f} kg'.format(hub.spinner.mass)
    print 'HUB TOTAL     {0:8.1f} kg'.format(hub.mass)
    print 'cm {0:6.2f} {1:6.2f} {2:6.2f}'.format(hub.cm[0], hub.cm[1], hub.cm[2])
    print 'I {0:6.1f} {1:6.1f} {2:6.1f}'.format(hub.I[0], hub.I[1], hub.I[2])

    # GRC 750 kW turbine
    print "windpact 750 kW turbine test"
    hub.bladeMass = 3400.0
    hub.rotorDiameter = 48.2
    hub.bladeNumer = 3
    hub.hubDiameter = 3.0
    AirDensity = 1.225
    Solidity = 0.07 # uknown value
    RatedWindSpeed = 16.0
    hub.rootMoment = (3.06 * pi / 8) * AirDensity * (RatedWindSpeed ** 2) * (Solidity * (hub.rotorDiameter ** 3)) / hub.bladeNumber

    hub.run()

    print "Hub Components"
    print '  hub         {0:8.1f} kg'.format(hub.hub.mass)
    print '  pitch mech  {0:8.1f} kg'.format(hub.pitchSystem.mass)
    print '  nose cone   {0:8.1f} kg'.format(hub.spinner.mass)
    print 'HUB TOTAL     {0:8.1f} kg'.format(hub.mass)
    print 'cm {0:6.2f} {1:6.2f} {2:6.2f}'.format(hub.cm[0], hub.cm[1], hub.cm[2])
    print 'I {0:6.1f} {1:6.1f} {2:6.1f}'.format(hub.I[0], hub.I[1], hub.I[2])

    # Windpact / Alstom 3.0 MW
    print "windpact / Alstom 3.0 MW"
    hub.bladeMass = 13363.0 # from CSM - not included in windpact/alstom data
    hub.rotorDiameter = 100.8 # alstom
    hub.bladeNumer = 3
    hub.hubDiameter = 0.0
    AirDensity = 1.225
    Solidity = 0.06 # unknown value
    RatedWindSpeed = 12.0 # expected to be closer to 1.5 and 5 MW
    hub.rootMoment = (3.06 * pi / 8) * AirDensity * (RatedWindSpeed ** 2) * (Solidity * (hub.rotorDiameter ** 3)) / hub.bladeNumber 

    hub.run()
    
    print "Hub Components"
    print '  hub         {0:8.1f} kg'.format(hub.hub.mass)
    print '  pitch mech  {0:8.1f} kg'.format(hub.pitchSystem.mass)
    print '  nose cone   {0:8.1f} kg'.format(hub.spinner.mass)
    print 'HUB TOTAL     {0:8.1f} kg'.format(hub.mass)
    print 'cm {0:6.2f} {1:6.2f} {2:6.2f}'.format(hub.cm[0], hub.cm[1], hub.cm[2])
    print 'I {0:6.1f} {1:6.1f} {2:6.1f}'.format(hub.I[0], hub.I[1], hub.I[2])

if __name__ == "__main__":

    example()
    
    example2()
