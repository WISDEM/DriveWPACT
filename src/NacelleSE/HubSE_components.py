"""
hubSE_components.py

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
    rotor_bending_moment = Float(iotype='in', units='N*m', desc='flapwise bending moment at blade root')
    rotor_diameter = Float(iotype='in', units='m', desc='rotor diameter')
    
    # parameters
    hub_diameter = Float(0.0, iotype='in', units='m', desc='hub diameter')
    blade_number = Int(3, iotype='in', desc='number of turbine blades')

    # outputs
    mass = Float(0.0, iotype='out', units='kg', desc='overall component mass')
    cm = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system')
    I = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass')
    
    def __init__(self):
        ''' 
        Initializes hub component 
        
        Parameters
        ----------
        rotor_bending_moment : float
          maximum flap-wise root moment for the blade (if == 0, then it is set within compute method) [Nm]
        rotor_diameter : float
          The wind turbine rotor diameter [m]
        hub_diameter : float
          the specified hub diameter (if == 0, then it is set within compute method) [m]
        blade_number : int
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

        #controls what happens if derivatives are missing
        self.missing_deriv_policy = 'assume_zero'

    def execute(self):

        # Sunderland method for calculating hub, pitch and spinner cone masses
        hubloadFact      = 1                                     # default is 3 blade rigid hub (should depend on hub type)
        hubgeomFact      = 1                                     # default is 3 blade (should depend on blade number)
        hubcontFact      = 1.5                                   # default is 1.5 for full span pitch control (should depend on control method)
        hubmatldensity   = 7860.0                               # density of hub material (kg / m^3) - assuming BS1503-622 (same material as LSS)
        hubmatlstress    = 371000000.0                                # allowable stress of hub material (N / m^2)

        # Root moment required as input, could be undone
        '''if rotor_bending_moment == 0.0:
            rotor_bending_moment = (3.06 * pi / 8) * AirDensity * (RatedWindSpeed ** 2) * (Solidity * (RotorDiam ** 3)) / BladeNum
                                                            # simplified equation for blade root moment (Sunderland model) if one is not provided'''
        
        self.mass =50 * hubgeomFact * hubloadFact * hubcontFact * self.blade_number * self.rotor_bending_moment * (hubmatldensity / hubmatlstress)
                                                            # mass of hub based on Sunderland model
                                                            # 31.4 adapted to 50 to fit 5 MW data

        # calculate mass properties
        if self.hub_diameter == 0:
            self.diameter =(3.30)
        else:
            self.diameter =(self.hub_diameter)
        self.thickness = self.diameter * (0.055 / 3.30)         # 0.055 for 1.5 MW outer diameter of 3.3 - using proportional constant

        cm = np.array([0.0,0.0,0.0])
        cm[0]     = - (0.05 * self.rotor_diameter)
        cm[1]     = 0.0
        cm[2]     = 0.025 * self.rotor_diameter
        self.cm = (cm)

        I = np.array([0.0, 0.0, 0.0])
        I[0] = 0.4 * (self.mass) * ((self.diameter / 2) ** 5 - (self.diameter / 2 - self.thickness) ** 5) / \
               ((self.diameter / 2) ** 3 - (self.diameter / 2 - self.thickness) ** 3)
        I[1] = I[0]
        I[2] = I[1]
        self.I = (I)

        # derivatives
        self.d_hubMass_d_rotor_bending_moment = 50 * hubgeomFact * hubloadFact * hubcontFact * self.blade_number * (hubmatldensity / hubmatlstress)
        self.d_cm_d_rotor_diameter = np.array([-0.05, 0.0, 0.025])
        self.d_I_d_rotor_bending_moment = 0.4 * (self.d_hubMass_d_rotor_bending_moment) * ((self.diameter / 2) ** 5 - (self.diameter / 2 - self.thickness) ** 5) / \
               ((self.diameter / 2) ** 3 - (self.diameter / 2 - self.thickness) ** 3)
        self.d_I_d_hub_diameter = 0.4 * self.mass * ((0.5**5 - (0.5 - 0.055/3.3)**5)/(0.5**3 - (0.5 - 0.055/3.3)**3)) * 2 * self.diameter
    
    def list_deriv_vars(self):

        inputs = ['rotor_bending_moment', 'rotor_diameter', 'hub_diameter']
        outputs = ['mass', 'cm', 'I']
        
        return inputs, outputs

    def provideJ(self):
  
        # Jacobian
        self.J = np.array([[self.d_hubMass_d_rotor_bending_moment, 0, 0], \
                           [0, self.d_cm_d_rotor_diameter[0], 0], \
                           [0, self.d_cm_d_rotor_diameter[1], 0], \
                           [0, self.d_cm_d_rotor_diameter[2], 0], \
                           [self.d_I_d_rotor_bending_moment, 0, self.d_I_d_hub_diameter], \
                           [self.d_I_d_rotor_bending_moment, 0, self.d_I_d_hub_diameter], \
                           [self.d_I_d_rotor_bending_moment, 0, self.d_I_d_hub_diameter]])

        return self.J

#-------------------------------------------------------------------------------

class PitchSystem(Component): 
    '''
     PitchSystem class          
      The PitchSystem class is used to represent the pitch system of a wind turbine. 
      It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
      It contains an update method to determine the mass, mass properties, and dimensions of the component.
    '''

    # variables
    blade_mass = Float(iotype='in', units='kg', desc='mass of one blade')
    rotor_bending_moment = Float(iotype='in', units='N*m', desc='flapwise bending moment at blade root')
    rotor_diameter = Float(iotype='in', units='m', desc='rotor diameter')
    
    # parameters
    hub_diameter = Float(0.0, iotype='in', units='m', desc='hub diameter')
    blade_number = Int(3, iotype='in', desc='number of turbine blades')

    # outputs
    mass = Float(0.0, iotype='out', units='kg', desc='overall component mass')
    cm = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system')
    I = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass')
    
    def __init__(self):
        ''' 
        Initializes pitch system
        
        Parameters
        ----------
        blade_mass : float
          The wind turbine individual blade mass [kg]
        rotor_bending_moment : float
          maximum flap-wise root moment for the blade (if == 0, then it is set within compute method) [Nm]
        rotor_diameter : float
          The wind turbine rotor diameter [m]
        hub_diameter : float
          the specified hub diameter (if == 0, then it is set within compute method) [m]
        blade_number : int
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

        #controls what happens if derivatives are missing
        self.missing_deriv_policy = 'assume_zero'

    def execute(self):

        # Sunderland method for calculating pitch system masses
        pitchmatldensity = 7860.0                             # density of pitch system material (kg / m^3) - assuming BS1503-622 (same material as LSS)
        pitchmatlstress  = 371000000.0                              # allowable stress of hub material (N / m^2)

        # Root moment required as input, could be undone
        '''if rotor_bending_moment == 0.0:
            rotor_bending_moment = (3.06 * pi / 8) * AirDensity * (RatedWindSpeed ** 2) * (Solidity * (RotorDiam ** 3)) / BladeNum
                                                            # simplified equation for blade root moment (Sunderland model) if one is not provided'''

        hubpitchFact      = 1.0                                 # default factor is 1.0 (0.54 for modern designs)
        #self.mass =hubpitchFact * (0.22 * self.blade_mass * self.blade_number + 12.6 * self.blade_number * self.rotor_bending_moment * (pitchmatldensity / pitchmatlstress))
                                                            # mass of pitch system based on Sunderland model
        self.mass =hubpitchFact * (0.22 * self.blade_mass * self.blade_number + 12.6 * self.rotor_bending_moment * (pitchmatldensity / pitchmatlstress))
                                                            # mass of pitch system based on Sunderland model


        # calculate mass properties
        if self.hub_diameter == 0:
            self.diameter =(3.30)
        else:
            self.diameter =(self.hub_diameter)

        # calculate mass properties
        cm = np.array([0.0,0.0,0.0])
        cm[0]     = - (0.05 * self.rotor_diameter)
        cm[1]     = 0.0
        cm[2]     = 0.025 * self.rotor_diameter
        self.cm = (cm)

        I = np.array([0.0, 0.0, 0.0])
        I[0] = self.mass * (self.diameter ** 2) / 4
        I[1] = I[0]
        I[2] = I[1]
        self.I = (I)

        # derivatives
        self.d_pitchMass_d_blade_mass = hubpitchFact * 0.22 * self.blade_number
        self.d_pitchMass_d_rotor_bending_moment = hubpitchFact * 12.6 * pitchmatldensity / pitchmatlstress
        self.d_cm_d_rotor_diameter = np.array([-0.05, 0.0, 0.025])
        self.d_I_d_blade_mass = self.d_pitchMass_d_blade_mass * (self.diameter**2) / 4
        self.d_I_d_rotor_bending_moment = self.d_pitchMass_d_rotor_bending_moment * (self.diameter**2) / 4
        self.d_I_d_hub_diameter = self.mass * (2./4.) * self.diameter


    def list_deriv_vars(self):

        inputs = ['blade_mass', 'rotor_bending_moment', 'rotor_diameter', 'hub_diameter']
        outputs = ['mass', 'cm', 'I']
        
        return inputs, outputs

    def provideJ(self):

        # Jacobian
        self.J = np.array([[self.d_pitchMass_d_blade_mass, self.d_pitchMass_d_rotor_bending_moment, 0, 0], \
                           [0, 0, self.d_cm_d_rotor_diameter[0], 0], \
                           [0, 0, self.d_cm_d_rotor_diameter[1], 0], \
                           [0, 0, self.d_cm_d_rotor_diameter[2], 0], \
                           [self.d_I_d_blade_mass, self.d_I_d_rotor_bending_moment, 0, self.d_I_d_hub_diameter], \
                           [self.d_I_d_blade_mass, self.d_I_d_rotor_bending_moment, 0, self.d_I_d_hub_diameter], \
                           [self.d_I_d_blade_mass, self.d_I_d_rotor_bending_moment, 0, self.d_I_d_hub_diameter]])

        return self.J

#-------------------------------------------------------------------------------      

class Spinner(Component): 
    ''' 
       Spinner class
          The SpinnerClass is used to represent the spinner of a wind turbine. 
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.
    '''
    
    # variables
    rotor_diameter = Float(iotype='in', units='m', desc='rotor diameter')

    # parameters
    hub_diameter = Float(0.0, iotype='in', units='m', desc='hub diameter')

    # outputs
    mass = Float(0.0, iotype='out', units='kg', desc='overall component mass')
    cm = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system')
    I = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass')
    
    def __init__(self):
        ''' 
        Initializes spinner system
        
        Parameters
        ----------
        rotor_diameter : float
          The wind turbine rotor diameter [m]
        hub_diameter : float
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

        #controls what happens if derivatives are missing
        self.missing_deriv_policy = 'assume_zero'

    def execute(self):

        self.mass =18.5 * self.rotor_diameter + (-520.5)   # spinner mass comes from cost and scaling model

        # calculate mass properties
        if self.hub_diameter == 0:
            self.diameter =(3.30)
        else:
            self.diameter =(self.hub_diameter)
        self.thickness = self.diameter * (0.055 / 3.30)         # 0.055 for 1.5 MW outer diameter of 3.3 - using proportional constant

        cm = np.array([0.0,0.0,0.0])
        cm[0]     = - (0.05 * self.rotor_diameter)
        cm[1]     = 0.0
        cm[2]     = 0.025 * self.rotor_diameter
        self.cm = (cm)

        I = np.array([0.0, 0.0, 0.0])
        I[0] = 0.4 * (self.mass) * ((self.diameter / 2) ** 5 - (self.diameter / 2 - self.thickness) ** 5) / \
               ((self.diameter / 2) ** 3 - (self.diameter / 2 - self.thickness) ** 3)
        I[1] = I[0]
        I[2] = I[1]
        self.I = (I)

        # derivatives
        self.d_spinnerMass_d_rotor_diameter = 18.5
        self.d_cm_d_rotor_diameter = np.array([-0.05, 0.0, 0.025])
        self.d_I_d_rotor_diameter = 0.4 * self.d_spinnerMass_d_rotor_diameter * ((self.diameter / 2) ** 5 - (self.diameter / 2 - self.thickness) ** 5) / \
               ((self.diameter / 2) ** 3 - (self.diameter / 2 - self.thickness) ** 3)
        self.d_I_d_hub_diameter = 0.4 * self.mass * ((0.5**5 - (0.5 - 0.055/3.3)**5)/(0.5**3 - (0.5 - 0.055/3.3)**3)) * 2 * self.diameter

    def list_deriv_vars(self):

        inputs = ['rotor_diameter', 'hub_diameter']
        outputs = ['mass', 'cm', 'I']

        return inputs, outputs

    def provideJ(self):

        # Jacobian
        self.J = np.array([[self.d_spinnerMass_d_rotor_diameter, 0], \
                           [self.d_cm_d_rotor_diameter[0], 0], \
                           [self.d_cm_d_rotor_diameter[1], 0], \
                           [self.d_cm_d_rotor_diameter[2], 0], \
                           [self.d_I_d_rotor_diameter, self.d_I_d_hub_diameter], \
                           [self.d_I_d_rotor_diameter, self.d_I_d_hub_diameter], \
                           [self.d_I_d_rotor_diameter, self.d_I_d_hub_diameter]])

        return self.J

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

        #controls what happens if derivatives are missing
        self.missing_deriv_policy = 'assume_zero'

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
        self.d_mass_d_hubMass = 1
        self.d_mass_d_pitchMass = 1
        self.d_mass_d_spinnerMass = 1

        self.d_cm_d_hubMass = np.array([0.0, 0.0, 0.0])
        for i in (range(0,3)):
            self.d_cm_d_hubMass[i] = (self.hubCM[i]*self.mass - (self.hubMass * self.hubCM[i] + self.pitchMass * self.pitchCM[i] + \
                    self.spinnerMass * self.spinnerCM[i] ))/ ((self.mass)**2)
        self.d_cm_d_pitchMass = np.array([0.0, 0.0, 0.0])
        for i in (range(0,3)):
            self.d_cm_d_pitchMass[i] = (self.pitchCM[i]*self.mass - (self.hubMass * self.hubCM[i] + self.pitchMass * self.pitchCM[i] + \
                    self.spinnerMass * self.spinnerCM[i] ))/ ((self.mass)**2)
        self.d_cm_d_spinnerMass = np.array([0.0, 0.0, 0.0])
        for i in (range(0,3)):
            self.d_cm_d_spinnerMass[i] = (self.spinnerCM[i]*self.mass - (self.hubMass * self.hubCM[i] + self.pitchMass * self.pitchCM[i] + \
                    self.spinnerMass * self.spinnerCM[i] ))/ ((self.mass)**2)

        self.d_cm_d_hubCM = self.hubMass / self.mass
        self.d_cm_d_pitchCM = self.pitchMass / self.mass
        self.d_cm_d_spinnerCM = self.spinnerMass / self.mass

        self.d_I_d_hubI = 1
        self.d_I_d_pitchI = 1
        self.d_I_d_spinnerI = 1
        
        self.d_I_d_hubMass = np.array([0.0, 0.0, 0.0])
        for i in (range(0,3)):
            self.d_I_d_hubMass[i] = (self.hubCM[i]-self.cm[i])**2 + 2 * self.hubMass * (self.hubCM[i] - self.cm[i]) * self.d_cm_d_hubCM        
        self.d_I_d_pitchMass = np.array([0.0, 0.0, 0.0])
        for i in (range(0,3)):
            self.d_I_d_pitchMass[i] = (self.pitchCM[i]-self.cm[i])**2 + 2 * self.pitchMass * (self.pitchCM[i] - self.cm[i]) * self.d_cm_d_pitchCM  
        self.d_I_d_spinnerMass = np.array([0.0, 0.0, 0.0])
        for i in (range(0,3)):
            self.d_I_d_spinnerMass[i] = (self.spinnerCM[i]-self.cm[i])**2 + 2 * self.spinnerMass * (self.spinnerCM[i] - self.cm[i]) * self.d_cm_d_spinnerCM

        self.d_I_d_hubCM = np.array([0.0, 0.0, 0.0])
        for i in range(0,3):
            self.d_I_d_hubCM[i] = 2 * self.hubMass * (self.hubCM[i] - self.cm[i]) * (1 - self.d_cm_d_hubCM)
        self.d_I_d_pitchCM = np.array([0.0, 0.0, 0.0])
        for i in range(0,3):
            self.d_I_d_pitchCM[i] = 2 * self.hubMass * (self.pitchCM[i] - self.cm[i]) * (1 - self.d_cm_d_pitchCM)
        self.d_I_d_spinnerCM = np.array([0.0, 0.0, 0.0])
        for i in range(0,3):
            self.d_I_d_spinnerCM[i] = 2 * self.spinnerMass * (self.spinnerCM[i] - self.cm[i]) * (1 - self.d_cm_d_spinnerCM)

    def list_deriv_vars(self):

        inputs = ['hubMass', 'pitchMass', 'spinnerMass', 'hubCM', 'pitchCM', 'spinnerCM',  'hubI', 'pitchI', 'spinnerI']
        outputs = ['mass', 'cm', 'I']
        
        return inputs, outputs

    def provideJ(self):

        # Jacobian
        self.J = np.array([[self.d_mass_d_hubMass, self.d_mass_d_pitchMass, self.d_mass_d_spinnerMass, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], \
                           [self.d_cm_d_hubMass[0], self.d_cm_d_pitchMass[0], self.d_cm_d_spinnerMass[0], self.d_cm_d_hubCM, 0, 0, self.d_cm_d_pitchCM, 0, 0, self.d_cm_d_spinnerCM, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], \
                           [self.d_cm_d_hubMass[1], self.d_cm_d_pitchMass[1], self.d_cm_d_spinnerMass[1], 0, self.d_cm_d_hubCM, 0, 0, self.d_cm_d_pitchCM, 0, 0, self.d_cm_d_spinnerCM, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], \
                           [self.d_cm_d_hubMass[2], self.d_cm_d_pitchMass[2], self.d_cm_d_spinnerMass[2], 0, 0, self.d_cm_d_hubCM, 0, 0, self.d_cm_d_pitchCM, 0, 0, self.d_cm_d_spinnerCM, 0, 0, 0, 0, 0, 0, 0, 0, 0], \
                           [self.d_I_d_hubMass[0], self.d_I_d_pitchMass[0], self.d_I_d_spinnerMass[0], self.d_I_d_hubCM[0], 0, 0, self.d_I_d_pitchCM[0], 0, 0, self.d_I_d_spinnerCM[0], 0, 0, self.d_I_d_hubI, 0, 0, self.d_I_d_pitchI, 0, 0, self.d_I_d_spinnerI, 0, 0], \
                           [self.d_I_d_hubMass[1], self.d_I_d_pitchMass[1], self.d_I_d_spinnerMass[1], 0, self.d_I_d_hubCM[1], 0, 0, self.d_I_d_pitchCM[1], 0, 0, self.d_I_d_spinnerCM[1], 0, 0, self.d_I_d_hubI, 0, 0, self.d_I_d_pitchI, 0, 0, self.d_I_d_spinnerI, 0], \
                           [self.d_I_d_hubMass[2], self.d_I_d_pitchMass[2], self.d_I_d_spinnerMass[2], 0, 0, self.d_I_d_hubCM[2], 0, 0, self.d_I_d_pitchCM[2], 0, 0, self.d_I_d_spinnerCM[2], 0, 0, self.d_I_d_hubI, 0, 0, self.d_I_d_pitchI, 0, 0, self.d_I_d_spinnerI]])

        return self.J

#--------------------------------------------------------------------------------------------------------

if __name__ == '__main__':
     pass