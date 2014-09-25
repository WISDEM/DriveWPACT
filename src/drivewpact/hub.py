"""
hubSE.py

Created by Katherine Dykes 2012.
Copyright (c) NREL. All rights reserved.
"""

from openmdao.main.api import Assembly, Component
from openmdao.main.datatypes.api import Float, Int, Array
from math import pi
import numpy as np

from fusedwind.interface import implement_base, base

#-----------------------------------------------------------------------------

# Hub Base Assembly 
@base
class HubBase(Assembly):

    # variables
    blade_mass = Float(iotype='in', units='kg', desc='mass of one blade')
    rotor_bending_moment = Float(iotype='in', units='N*m', desc='flapwise bending moment at blade root')
    rotor_diameter = Float(iotype='in', units='m', desc='rotor diameter')
    blade_root_diameter = Float(iotype='in', units='m', desc='blade root diameter')

    # parameters
    blade_number = Int(3, iotype='in', desc='number of turbine blades')

    # outputs
    hub_system_mass = Float(0.0, iotype='out', units='kg', desc='overall component mass')
    hub_system_cm = Array(iotype='out', desc='center of mass of the hub relative to tower to in yaw-aligned c.s.')
    hub_system_I = Array(iotype='out', desc='mass moments of Inertia of hub [Ixx, Iyy, Izz, Ixy, Ixz, Iyz] around its center of mass in yaw-aligned c.s.')

    hub_mass = Float(0.0, iotype='out', units='kg')
    pitch_system_mass = Float(0.0, iotype='out', units='kg')
    spinner_mass = Float(0.0, iotype='out', units='kg')

# Hub Assembly and Components
@implement_base(HubBase)
class HubWPACT(Assembly):
    '''
       HubWPACT class
          The HubWPACT class is used to represent the hub system of a wind turbine.
    '''

    # variables
    blade_mass = Float(iotype='in', units='kg', desc='mass of one blade')
    rotor_bending_moment = Float(iotype='in', units='N*m', desc='flapwise bending moment at blade root')
    rotor_diameter = Float(iotype='in', units='m', desc='rotor diameter')
    blade_root_diameter = Float(iotype='in', units='m', desc='blade root diameter')

    # parameters
    blade_number = Int(3, iotype='in', desc='number of turbine blades')

    # outputs
    hub_system_mass = Float(0.0, iotype='out', units='kg', desc='overall component mass')
    hub_system_cm = Array(iotype='out', desc='center of mass of the hub relative to tower to in yaw-aligned c.s.')
    hub_system_I = Array(iotype='out', desc='mass moments of Inertia of hub [Ixx, Iyy, Izz, Ixy, Ixz, Iyz] around its center of mass in yaw-aligned c.s.')

    hub_mass = Float(0.0, iotype='out', units='kg')
    pitch_system_mass = Float(0.0, iotype='out', units='kg')
    spinner_mass = Float(0.0, iotype='out', units='kg')

    def configure(self):

        # select components
        self.add('hubSystem', HubSystemAdder())
        self.add('hub', Hub())
        self.add('pitchSystem', PitchSystem())
        self.add('spinner', Spinner())

        # workflow
        self.driver.workflow.add(['hubSystem', 'hub', 'pitchSystem', 'spinner'])

        # connect inputs
        self.connect('blade_mass', ['pitchSystem.blade_mass'])
        self.connect('rotor_bending_moment', ['hub.rotor_bending_moment', 'pitchSystem.rotor_bending_moment'])
        self.connect('blade_number', ['hub.blade_number', 'pitchSystem.blade_number'])
        self.connect('rotor_diameter', ['hub.rotor_diameter', 'pitchSystem.rotor_diameter', 'spinner.rotor_diameter'])
        self.connect('blade_root_diameter', ['hub.hub_diameter', 'pitchSystem.hub_diameter', 'spinner.hub_diameter'])

        # connect components
        self.connect('hub.mass', 'hubSystem.hub_mass')
        self.connect('hub.cm', 'hubSystem.hub_cm')
        self.connect('hub.I', 'hubSystem.hub_I')
        self.connect('pitchSystem.mass', 'hubSystem.pitch_system_mass')
        self.connect('pitchSystem.cm', 'hubSystem.pitch_system_cm')
        self.connect('pitchSystem.I', 'hubSystem.pitch_system_I')
        self.connect('spinner.mass', 'hubSystem.spinner_mass')
        self.connect('spinner.cm', 'hubSystem.spinner_cm')
        self.connect('spinner.I', 'hubSystem.spinner_I')

        # connect outputs
        self.connect('hubSystem.hub_system_mass', 'hub_system_mass')
        self.connect('hubSystem.hub_system_cm', 'hub_system_cm')
        self.connect('hubSystem.hub_system_I', 'hub_system_I')
        self.connect('hub.mass', 'hub_mass')
        self.connect('pitchSystem.mass', 'pitch_system_mass')
        self.connect('spinner.mass', 'spinner_mass')

#-------------------------------------------------------------------------------

# Hub System Component Models

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

        I = np.zeros(3)
        I[0] = 0.4 * (self.mass) * ((self.diameter / 2.) ** 5 - (self.diameter / 2. - self.thickness) ** 5) / \
               ((self.diameter / 2.) ** 3 - (self.diameter / 2. - self.thickness) ** 3)
        I[1] = I[0]
        I[2] = I[1]
        self.I = I

        # derivatives
        self.d_hub_mass_d_rotor_bending_moment = 50 * hubgeomFact * hubloadFact * hubcontFact * self.blade_number * (hubmatldensity / hubmatlstress)
        self.d_cm_d_rotor_diameter = np.array([-0.05, 0.0, 0.025])
        self.d_I_d_rotor_bending_moment = 0.4 * (self.d_hub_mass_d_rotor_bending_moment) * ((self.diameter / 2.) ** 5 - (self.diameter / 2. - self.thickness) ** 5) / \
               ((self.diameter / 2.) ** 3 - (self.diameter / 2. - self.thickness) ** 3)
        self.d_I_d_hub_diameter = 0.4 * self.mass * ((0.5**5 - (0.5 - 0.055/3.3)**5)/(0.5**3 - (0.5 - 0.055/3.3)**3)) * 2 * self.diameter

    def list_deriv_vars(self):

        inputs = ['rotor_bending_moment', 'rotor_diameter', 'hub_diameter']
        outputs = ['mass', 'cm', 'I']

        return inputs, outputs

    def provideJ(self):

        # Jacobian
        self.J = np.array([[self.d_hub_mass_d_rotor_bending_moment, 0, 0], \
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
        self.d_pitch_system_mass_d_blade_mass = hubpitchFact * 0.22 * self.blade_number
        self.d_pitch_system_mass_d_rotor_bending_moment = hubpitchFact * 12.6 * pitchmatldensity / pitchmatlstress
        self.d_cm_d_rotor_diameter = np.array([-0.05, 0.0, 0.025])
        self.d_I_d_blade_mass = self.d_pitch_system_mass_d_blade_mass * (self.diameter**2) / 4
        self.d_I_d_rotor_bending_moment = self.d_pitch_system_mass_d_rotor_bending_moment * (self.diameter**2) / 4
        self.d_I_d_hub_diameter = self.mass * (2./4.) * self.diameter


    def list_deriv_vars(self):

        inputs = ['blade_mass', 'rotor_bending_moment', 'rotor_diameter', 'hub_diameter']
        outputs = ['mass', 'cm', 'I']

        return inputs, outputs

    def provideJ(self):

        # Jacobian
        self.J = np.array([[self.d_pitch_system_mass_d_blade_mass, self.d_pitch_system_mass_d_rotor_bending_moment, 0, 0], \
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
        self.d_spinner_mass_d_rotor_diameter = 18.5
        self.d_cm_d_rotor_diameter = np.array([-0.05, 0.0, 0.025])
        self.d_I_d_rotor_diameter = 0.4 * self.d_spinner_mass_d_rotor_diameter * ((self.diameter / 2) ** 5 - (self.diameter / 2 - self.thickness) ** 5) / \
               ((self.diameter / 2) ** 3 - (self.diameter / 2 - self.thickness) ** 3)
        self.d_I_d_hub_diameter = 0.4 * self.mass * ((0.5**5 - (0.5 - 0.055/3.3)**5)/(0.5**3 - (0.5 - 0.055/3.3)**3)) * 2 * self.diameter

    def list_deriv_vars(self):

        inputs = ['rotor_diameter', 'hub_diameter']
        outputs = ['mass', 'cm', 'I']

        return inputs, outputs

    def provideJ(self):

        # Jacobian
        self.J = np.array([[self.d_spinner_mass_d_rotor_diameter, 0], \
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
    hub_mass = Float(iotype='in', units='kg', desc='overall component mass')
    hub_cm = Array(np.array([0.0, 0.0, 0.0]), iotype='in', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system')
    hub_I = Array(np.array([0.0, 0.0, 0.0]), iotype='in', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass')
    pitch_system_mass = Float(iotype='in', units='kg', desc='overall component mass')
    pitch_system_cm = Array(np.array([0.0, 0.0, 0.0]), iotype='in', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system')
    pitch_system_I = Array(np.array([0.0, 0.0, 0.0]), iotype='in', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass')
    spinner_mass = Float(iotype='in', units='kg', desc='overall component mass')
    spinner_cm = Array(np.array([0.0, 0.0, 0.0]), iotype='in', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system')
    spinner_I = Array(np.array([0.0, 0.0, 0.0]), iotype='in', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass')

    # outputs
    hub_system_mass = Float(0.0, iotype='out', units='kg', desc='overall component mass')
    hub_system_cm = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system')
    hub_system_I = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass')


    def __init__(self):
        '''
        Initializes hub system component
        '''

        super(HubSystemAdder, self).__init__()

        #controls what happens if derivatives are missing
        self.missing_deriv_policy = 'assume_zero'

    def execute(self):

        self.hub_system_mass = self.hub_mass + self.pitch_system_mass + self.spinner_mass

        # calculate mass properties
        # calculation of mass center and moments of inertia
        cm = np.array([0.0,0.0,0.0])
        for i in (range(0,3)):
            # calculate center of mass
            cm[i] = (self.hub_mass * self.hub_cm[i] + self.pitch_system_mass * self.pitch_system_cm[i] + \
                    self.spinner_mass * self.spinner_cm[i] ) / (self.hub_system_mass)
        self.hub_system_cm = cm

        I = np.zeros(6)
        for i in (range(0,3)):                        # calculating MOI, at nacelle center of gravity with origin at tower top center / yaw mass center, ignoring masses of non-drivetrain components / auxiliary systems
            # calculate moments around CM
            # sum moments around each components CM
            I[i]  =  self.hub_I[i] + self.pitch_system_I[i] + self.spinner_I[i]
            # translate to hub system CM using parallel axis theorem
            for j in (range(0,3)):
                if i != j:
                    I[i] +=  (self.hub_mass * (self.hub_cm[j] - self.hub_system_cm[j]) ** 2) + \
                                  (self.pitch_system_mass * (self.pitch_system_cm[j] - self.hub_system_cm[j]) ** 2) + \
                                  (self.spinner_mass * (self.spinner_cm[j] - self.hub_system_cm[j]) ** 2)
        self.hub_system_I = I

        # derivatives
        self.d_mass_d_hub_mass = 1
        self.d_mass_d_pitch_system_mass = 1
        self.d_mass_d_spinner_mass = 1

        self.d_cm_d_hub_mass = np.array([0.0, 0.0, 0.0])
        for i in (range(0,3)):
            self.d_cm_d_hub_mass[i] = (self.hub_cm[i]*self.hub_system_mass - (self.hub_mass * self.hub_cm[i] + self.pitch_system_mass * self.pitch_system_cm[i] + \
                    self.spinner_mass * self.spinner_cm[i] ))/ ((self.hub_system_mass)**2)
        self.d_cm_d_pitch_system_mass = np.array([0.0, 0.0, 0.0])
        for i in (range(0,3)):
            self.d_cm_d_pitch_system_mass[i] = (self.pitch_system_cm[i]*self.hub_system_mass - (self.hub_mass * self.hub_cm[i] + self.pitch_system_mass * self.pitch_system_cm[i] + \
                    self.spinner_mass * self.spinner_cm[i] ))/ ((self.hub_system_mass)**2)
        self.d_cm_d_spinner_mass = np.array([0.0, 0.0, 0.0])
        for i in (range(0,3)):
            self.d_cm_d_spinner_mass[i] = (self.spinner_cm[i]*self.hub_system_mass - (self.hub_mass * self.hub_cm[i] + self.pitch_system_mass * self.pitch_system_cm[i] + \
                    self.spinner_mass * self.spinner_cm[i] ))/ ((self.hub_system_mass)**2)

        self.d_cm_d_hub_cm = self.hub_mass / self.hub_system_mass
        self.d_cm_d_pitch_system_cm = self.pitch_system_mass / self.hub_system_mass
        self.d_cm_d_spinner_cm = self.spinner_mass / self.hub_system_mass

        self.d_I_d_hub_I = 1
        self.d_I_d_pitch_system_I = 1
        self.d_I_d_spinner_I = 1

        self.d_I_d_hub_mass = np.array([0.0, 0.0, 0.0])
        for i in range(len(self.d_I_d_hub_mass)):
            for j in range(len(self.d_I_d_hub_mass)):
                if i != j:
                  self.d_I_d_hub_mass[i] += (self.hub_cm[j]-self.hub_system_cm[j])**2 \
                                           - 2 * self.hub_mass * (self.hub_cm[j] - self.hub_system_cm[j]) * (self.d_cm_d_hub_mass[j]) \
                                           - 2 * self.pitch_system_mass * (self.pitch_system_cm[j] - self.hub_system_cm[j]) * (self.d_cm_d_hub_mass[j]) \
                                           - 2 * self.spinner_mass * (self.spinner_cm[j] - self.hub_system_cm[j]) * (self.d_cm_d_hub_mass[j])
        self.d_I_d_pitch_system_mass = np.array([0.0, 0.0, 0.0])
        for i in range(len(self.d_I_d_pitch_system_mass)):
            for j in range(len(self.d_I_d_pitch_system_mass)):
                if i != j:
                  self.d_I_d_pitch_system_mass[i] += (self.pitch_system_cm[j]-self.hub_system_cm[j])**2 \
                                           - 2 * self.hub_mass * (self.hub_cm[j] - self.hub_system_cm[j]) * (self.d_cm_d_pitch_system_mass[j]) \
                                           - 2 * self.pitch_system_mass * (self.pitch_system_cm[j] - self.hub_system_cm[j]) * (self.d_cm_d_pitch_system_mass[j]) \
                                           - 2 * self.spinner_mass * (self.spinner_cm[j] - self.hub_system_cm[j]) * (self.d_cm_d_pitch_system_mass[j])
        self.d_I_d_spinner_mass = np.array([0.0, 0.0, 0.0])
        for i in range(len(self.d_I_d_spinner_mass)):
            for j in range(len(self.d_I_d_spinner_mass)):
                if i != j:
                  self.d_I_d_spinner_mass[i] += (self.spinner_cm[j]-self.hub_system_cm[j])**2 \
                                           - 2 * self.hub_mass * (self.hub_cm[j] - self.hub_system_cm[j]) * (self.d_cm_d_spinner_mass[j]) \
                                           - 2 * self.pitch_system_mass * (self.pitch_system_cm[j] - self.hub_system_cm[j]) * (self.d_cm_d_spinner_mass[j]) \
                                           - 2 * self.spinner_mass * (self.spinner_cm[j] - self.hub_system_cm[j]) * (self.d_cm_d_spinner_mass[j])

        self.d_I_d_hub_cm = np.zeros((3,3))
        for i in range(len(self.d_I_d_hub_cm)):
            for j in range(len(self.d_I_d_hub_cm)):
                if i != j:
                    self.d_I_d_hub_cm[i,j] = 2 * self.hub_mass * (self.hub_cm[j] - self.hub_system_cm[j]) * (1 - self.d_cm_d_hub_cm) \
                                           - 2 * self.pitch_system_mass * (self.pitch_system_cm[j] - self.hub_system_cm[j]) * (self.d_cm_d_hub_cm) \
                                           - 2 * self.spinner_mass * (self.spinner_cm[j] - self.hub_system_cm[j]) * (self.d_cm_d_hub_cm)
        self.d_I_d_pitch_system_cm = np.zeros((3,3))
        for i in range(len(self.d_I_d_pitch_system_cm)):
            for j in (range(len(self.d_I_d_pitch_system_cm))):
                if i != j:
                    self.d_I_d_pitch_system_cm[i,j] = - 2 * self.hub_mass * (self.hub_cm[j] - self.hub_system_cm[j]) * (self.d_cm_d_pitch_system_cm) \
                                           + 2 * self.pitch_system_mass * (self.pitch_system_cm[j] - self.hub_system_cm[j]) * (1 - self.d_cm_d_pitch_system_cm) \
                                           - 2 * self.spinner_mass * (self.spinner_cm[j] - self.hub_system_cm[j]) * (self.d_cm_d_pitch_system_cm)
        self.d_I_d_spinner_cm = np.zeros((3,3))
        for i in range(len(self.d_I_d_spinner_cm)):
            for j in (range(len(self.d_I_d_spinner_cm))):
                if i != j:
                    self.d_I_d_spinner_cm[i,j] = - 2 * self.hub_mass * (self.hub_cm[j] - self.hub_system_cm[j]) * (self.d_cm_d_spinner_cm) \
                                           - 2 * self.pitch_system_mass * (self.pitch_system_cm[j] - self.hub_system_cm[j]) * (self.d_cm_d_spinner_cm) \
                                           + 2 * self.spinner_mass * (self.spinner_cm[j] - self.hub_system_cm[j]) * (1 - self.d_cm_d_spinner_cm)
    def list_deriv_vars(self):

        inputs = ['hub_mass', 'pitch_system_mass', 'spinner_mass', 'hub_cm', 'pitch_system_cm', 'spinner_cm',  'hub_I', 'pitch_system_I', 'spinner_I']
        outputs = ['hub_system_mass', 'hub_system_cm', 'hub_system_I']

        return inputs, outputs

    def provideJ(self):

        # Jacobian
        self.J = np.array([[self.d_mass_d_hub_mass, self.d_mass_d_pitch_system_mass, self.d_mass_d_spinner_mass, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], \
                           [self.d_cm_d_hub_mass[0], self.d_cm_d_pitch_system_mass[0], self.d_cm_d_spinner_mass[0], self.d_cm_d_hub_cm, 0, 0, self.d_cm_d_pitch_system_cm, 0, 0, self.d_cm_d_spinner_cm, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], \
                           [self.d_cm_d_hub_mass[1], self.d_cm_d_pitch_system_mass[1], self.d_cm_d_spinner_mass[1], 0, self.d_cm_d_hub_cm, 0, 0, self.d_cm_d_pitch_system_cm, 0, 0, self.d_cm_d_spinner_cm, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], \
                           [self.d_cm_d_hub_mass[2], self.d_cm_d_pitch_system_mass[2], self.d_cm_d_spinner_mass[2], 0, 0, self.d_cm_d_hub_cm, 0, 0, self.d_cm_d_pitch_system_cm, 0, 0, self.d_cm_d_spinner_cm, 0, 0, 0, 0, 0, 0, 0, 0, 0], \
                           [self.d_I_d_hub_mass[0], self.d_I_d_pitch_system_mass[0], self.d_I_d_spinner_mass[0], self.d_I_d_hub_cm[0,0], self.d_I_d_hub_cm[0,1], self.d_I_d_hub_cm[0,2], self.d_I_d_pitch_system_cm[0,0], self.d_I_d_pitch_system_cm[0,1], self.d_I_d_pitch_system_cm[0,2], self.d_I_d_spinner_cm[0,0], self.d_I_d_spinner_cm[0,1], self.d_I_d_spinner_cm[0,2], self.d_I_d_hub_I, 0, 0, self.d_I_d_pitch_system_I, 0, 0, self.d_I_d_spinner_I, 0, 0], \
                           [self.d_I_d_hub_mass[1], self.d_I_d_pitch_system_mass[1], self.d_I_d_spinner_mass[1], self.d_I_d_hub_cm[1,0], self.d_I_d_hub_cm[1,1], self.d_I_d_hub_cm[1,2], self.d_I_d_pitch_system_cm[1,0], self.d_I_d_pitch_system_cm[1,1], self.d_I_d_pitch_system_cm[1,2], self.d_I_d_spinner_cm[1,0], self.d_I_d_spinner_cm[1,1], self.d_I_d_spinner_cm[1,2], 0, self.d_I_d_hub_I, 0, 0, self.d_I_d_pitch_system_I, 0, 0, self.d_I_d_spinner_I, 0], \
                           [self.d_I_d_hub_mass[2], self.d_I_d_pitch_system_mass[2], self.d_I_d_spinner_mass[2], self.d_I_d_hub_cm[2,0], self.d_I_d_hub_cm[2,1], self.d_I_d_hub_cm[2,2], self.d_I_d_pitch_system_cm[2,0], self.d_I_d_pitch_system_cm[2,1], self.d_I_d_pitch_system_cm[2,2], self.d_I_d_spinner_cm[2,0], self.d_I_d_spinner_cm[2,1], self.d_I_d_spinner_cm[2,2], 0, 0, self.d_I_d_hub_I, 0, 0, self.d_I_d_pitch_system_I, 0, 0, self.d_I_d_spinner_I], \
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], \
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], \
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
        return self.J

#-------------------------------------------------------------------------------

def example():

    # simple test of module

    # NREL 5 MW turbine
    print "NREL 5 MW turbine test"
    hub = HubWPACT()
    hub.blade_mass = 17740.0 # kg
    hub.rotor_diameter = 126.0 # m
    hub.blade_number  = 3
    hub.blade_root_diameter   = 3.542
    AirDensity= 1.225 # kg/(m^3)
    Solidity  = 0.0517
    RatedWindSpeed = 11.05 # m/s
    hub.rotor_bending_moment = (3.06 * pi / 8) * AirDensity * (RatedWindSpeed ** 2) * (Solidity * (hub.rotor_diameter ** 3)) / hub.blade_number

    hub.run()

    print "Hub Components"
    print '  hub         {0:8.1f} kg'.format(hub.hub.mass)  # 31644.47
    print '  pitch mech  {0:8.1f} kg'.format(hub.pitchSystem.mass) # 17003.98
    print '  nose cone   {0:8.1f} kg'.format(hub.spinner.mass) # 1810.50
    print 'Hub system total {0:8.1f} kg'.format(hub.hub_system_mass) # 50458.95
    print '    cm {0:6.2f} {1:6.2f} {2:6.2f}'.format(hub.hub_system_cm[0], hub.hub_system_cm[1], hub.hub_system_cm[2])
    print '    I {0:6.1f} {1:6.1f} {2:6.1f}'.format(hub.hub_system_I[0], hub.hub_system_I[1], hub.hub_system_I[2])
    print

def example2():

    # WindPACT 1.5 MW turbine
    print "WindPACT 1.5 MW turbine test"
    hub = HubWPACT()
    hub.blade_mass = 4470.0
    hub.rotor_diameter = 70.0
    hub.bladeNumer  = 3
    hub.hub_diameter   = 3.0
    AirDensity= 1.225
    Solidity  = 0.065
    RatedWindSpeed = 12.12
    hub.rotor_bending_moment = (3.06 * pi / 8) * AirDensity * (RatedWindSpeed ** 2) * (Solidity * (hub.rotor_diameter ** 3)) / hub.blade_number

    hub.run()

    print "Hub Components"
    print '  hub         {0:8.1f} kg'.format(hub.hub.mass)
    print '  pitch mech  {0:8.1f} kg'.format(hub.pitchSystem.mass)
    print '  nose cone   {0:8.1f} kg'.format(hub.spinner.mass)
    print 'HUB TOTAL     {0:8.1f} kg'.format(hub.hub_system_mass)
    print 'cm {0:6.2f} {1:6.2f} {2:6.2f}'.format(hub.hub_system_cm[0], hub.hub_system_cm[1], hub.hub_system_cm[2])
    print 'I {0:6.1f} {1:6.1f} {2:6.1f}'.format(hub.hub_system_I[0], hub.hub_system_I[1], hub.hub_system_I[2])
    print

    # GRC 750 kW turbine
    print "windpact 750 kW turbine test"
    hub.blade_mass = 3400.0
    hub.rotor_diameter = 48.2
    hub.bladeNumer = 3
    hub.hub_diameter = 3.0
    AirDensity = 1.225
    Solidity = 0.07 # uknown value
    RatedWindSpeed = 16.0
    hub.rotor_bending_moment = (3.06 * pi / 8) * AirDensity * (RatedWindSpeed ** 2) * (Solidity * (hub.rotor_diameter ** 3)) / hub.blade_number

    hub.run()

    print "Hub Components"
    print '  hub         {0:8.1f} kg'.format(hub.hub.mass)
    print '  pitch mech  {0:8.1f} kg'.format(hub.pitchSystem.mass)
    print '  nose cone   {0:8.1f} kg'.format(hub.spinner.mass)
    print 'HUB TOTAL     {0:8.1f} kg'.format(hub.hub_system_mass)
    print 'cm {0:6.2f} {1:6.2f} {2:6.2f}'.format(hub.hub_system_cm[0], hub.hub_system_cm[1], hub.hub_system_cm[2])
    print 'I {0:6.1f} {1:6.1f} {2:6.1f}'.format(hub.hub_system_I[0], hub.hub_system_I[1], hub.hub_system_I[2])
    print

if __name__ == "__main__":

    example()

    example2()
