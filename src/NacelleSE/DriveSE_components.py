"""
driveSE_components.py
New components for hub, low speed shaft, main bearings, gearbox, bedplate and yaw bearings

Created by Ryan King 2013.
Copyright (c) NREL. All rights reserved.
"""

from openmdao.main.api import Component, Assembly
from openmdao.main.datatypes.api import Float, Bool, Int, Str, Array
import numpy as np
from math import pi, cos, sqrt, radians, sin
import algopy
import scipy as scp
import scipy.optimize as opt

# -------------------------------------------------

class Hub_drive(Component):
    ''' Hub class    
          The Hub class is used to represent the hub component of a wind turbine. 
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.            
    '''

    # variables
    rotor_diameter = Float(iotype='in', units='m', desc='rotor diameter')
    blade_root_diameter = Float(iotype='in', units='m', desc='blade root diameter')
    
    # parameters
    blade_number = Int(3, iotype='in', desc='number of turbine blades')

    # outputs
    diameter = Float(0.0, iotype='out', units='m', desc='hub diameter')
    mass = Float(0.0, iotype='out', units='kg', desc='overall component mass')
    cm = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system')
    I = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass')
    
    def __init__(self):
        ''' 
        Initializes hub component 
        
        Parameters
        ----------
        rotor_diameter : float
          The wind turbine rotor diameter [m]
        blade_root_diameter : float
          The diameter of the blade root [m]
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

        super(Hub_drive, self).__init__()

    def execute(self):

        #Model hub as a cyclinder with holes for blade root and nacelle flange.
        rCyl=1.1*(self.blade_root_diameter/2) # TODO: sensitivity to this parameter
        hCyl=rCyl*2.8 # TODO: sensitivity to this parameter
        castThickness = rCyl/10.0 # TODO: sensitivity to this parameter
        approxCylVol=2*pi*rCyl*castThickness*hCyl
        bladeRootVol=pi*(self.blade_root_diameter/2.0)**2*castThickness

        #assume nacelle flange opening is similar to blade root opening
        approxCylNetVol = approxCylVol - (1.0 + self.blade_number)*bladeRootVol
        castDensity = 7200.0 # kg/m^3
        self.mass=approxCylNetVol*castDensity

        # calculate mass properties
        self.diameter=2*rCyl
        self.thickness=castThickness
                    
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
'''        d_hubMass_d_rootMoment = 50 * hubgeomFact * hubloadFact * hubcontFact * self.blade_number * (hubmatldensity / hubmatlstress)
        d_cm_d_rotor_diameter = np.array([-0.05, 0.0, 0.025])
        d_I_d_rootMoment = 0.4 * d_hubMass_d_rootMoment * ((0.5**5 - (0.05 - 0.055/3.3)**5)/(0.5**3 - (0.05 - 0.055/3.3)**3)) * self.diameter**2
        d_I_d_hubDiameter = 2 * 0.4 * self.mass * ((0.5**5 - (0.05 - 0.055/3.3)**5)/(0.5**3 - (0.05 - 0.055/3.3)**3)) * self.diameter
        
        # Jacobian
        self.J = np.array([[d_hubMass_d_rootMoment, 0, 0], \
                           [0, d_cm_d_rotor_diameter[0], 0], \
                           [0, d_cm_d_rotor_diameter[1], 0], \
                           [0, d_cm_d_rotor_diameter[2], 0], \
                           [d_I_d_rootMoment, 0, d_I_d_hubDiameter], \
                           [d_I_d_rootMoment, 0, d_I_d_hubDiameter], \
                           [d_I_d_rootMoment, 0, d_I_d_hubDiameter]])

    def provideJ(self):

        input_keys = ['rootMoment', 'rotor_diameter', 'hubDiameter']
        output_keys = ['hubMass', 'cm[0]', 'cm[1]', 'cm[2]', 'I[0]', 'I[1]', 'I[2]']

        self.derivatives.set_first_derivative(input_keys, output_keys, self.J)'''

# Resize shaft for bearing selection
def resize_for_bearings(D_mb, mbtype):
    # Internal function to resize shaft for bearings - for Yi to add content (using lookup table etc)   
    # To add bearing load capacity check later        

    # Modified by TParsons 4/21/14
    if D_mb <= 0.3:
      if mbtype == 'CARB':
              D_mb_a = 0.3 #m
              FW_mb = 0.16
      elif mbtype == 'SRB':
              D_mb_a = 0.3 #m
              FW_mb = 0.16
      elif mbtype == 'TRB1':
              D_mb_a = 0.3 #m
              FW_mb = 0.141      
      elif mbtype == 'CRB':
              D_mb_a = 0.3 #m
              FW_mb = 0.085 
      elif mbtype == 'RB':
              D_mb_a = 0.3 #m
              FW_mb = 0.074        
      elif mbtype == 'TRB2':
              D_mb_a = 0.3 #m
              FW_mb = 0.203  

    elif D_mb <= 0.34:
      if mbtype == 'CARB':
              D_mb_a = 0.34 #m
              FW_mb = 0.19
      elif mbtype == 'SRB':
              D_mb_a =  0.34 
              FW_mb =0.19
      elif mbtype == 'TRB1':
              D_mb_a = 0.34 #m
              FW_mb = 0.076    
      elif mbtype == 'CRB':
              D_mb_a = 0.34 #m
              FW_mb = 0.082 
      elif mbtype == 'RB':
              D_mb_a = 0.34 #m
              FW_mb = 0.082
      elif mbtype == 'TRB2':
            D_mb_a = 0.34 #m
            FW_mb = 0.16

    elif D_mb <= 0.38:
      D_mb_a = 0.38
      if mbtype == 'CARB': 
              FW_mb = 0.194
      elif mbtype == 'SRB':
              FW_mb = 0.243
      elif mbtype == 'TRB1':
              FW_mb = 0.050     
      elif mbtype == 'CRB':
              FW_mb = 0.106 
      elif mbtype == 'RB':
              FW_mb = 0.057      
      elif mbtype == 'TRB2':
              FW_mb = 0.148  

    elif D_mb <= 0.4:
      D_mb_a = 0.4
      if mbtype == 'CARB':
              FW_mb = 0.2
      elif mbtype == 'SRB':
              FW_mb =0.2
      elif mbtype == 'TRB1':
             D_mb_a = .4064
             FW_mb = 0.181     
      elif mbtype == 'CRB':
             FW_mb = 0.118    
      elif mbtype == 'RB':
             FW_mb = .044      
      elif mbtype == 'TRB2':
             D_mb_a = .4064
             FW_mb = 0.1429

    elif D_mb <= 0.44:
      D_mb_a = 0.44
      if mbtype == 'CARB':
             FW_mb = 0.226
      elif mbtype == 'SRB':
             FW_mb =0.226
      elif mbtype == 'TRB1':
             D_mb_a = .4477
             FW_mb = 0.12065     
      elif mbtype == 'CRB':
             FW_mb = 0.122    
      elif mbtype == 'RB':
             FW_mb = .080      
      elif mbtype == 'TRB2':
             D_mb_a = 0.440
             FW_mb = 0.196

    elif D_mb <= 0.48:
      D_mb_a = 0.48
      if mbtype == 'CARB':
             FW_mb = 0.248
      elif mbtype == 'SRB':
             FW_mb =0.248
      elif mbtype == 'TRB1':
             D_mb_a = .4794
             FW_mb = 0.1365    
      elif mbtype == 'CRB':
             FW_mb = 0.128    
      elif mbtype == 'RB':
             FW_mb = 0.078      
      elif mbtype == 'TRB2':
             D_mb_a = 0.4794
             FW_mb = 0.2762

    elif D_mb <= 0.5:
      D_mb_a = 0.5
      if mbtype == 'CARB':
             FW_mb = 0.167
      elif mbtype == 'SRB':
             FW_mb = 0.167
      elif mbtype == 'TRB1':
             D_mb_a = .4985
             FW_mb = 0.081    
      elif mbtype == 'CRB':
             FW_mb = 0.128    
      elif mbtype == 'RB':
             FW_mb = 0.078      
      elif mbtype == 'TRB2':
             D_mb_a = 0.50165
             FW_mb = 0.2921

    elif D_mb <= 0.56:
      D_mb_a = 0.56
      if mbtype == 'CARB':
             FW_mb = 0.195
      elif mbtype == 'SRB':
             FW_mb = 0.195
      elif mbtype == 'TRB1':
             D_mb_a = .5588
             FW_mb = 0.1048   
      elif mbtype == 'CRB':
             FW_mb = 0.112   
      elif mbtype == 'RB':
             FW_mb = 0.085      
      elif mbtype == 'TRB2':
             D_mb_a = 0.5588
             FW_mb = 0.2254

    elif D_mb <= 0.60:
      D_mb_a = 0.60
      if mbtype == 'CARB':
             FW_mb = 0.20
      elif mbtype == 'SRB':
             FW_mb = 0.20
      elif mbtype == 'TRB1':
             D_mb_a = .60772
             FW_mb = 0.0937 
      elif mbtype == 'CRB':
             FW_mb = 0.118   
      elif mbtype == 'RB':
             FW_mb = 0.090      
      elif mbtype == 'TRB2':
             D_mb_a = 0.6029
             FW_mb = 0.2064

    elif D_mb <= 0.67:
      D_mb_a = 0.67
      if mbtype == 'CARB':
             FW_mb = 0.230
      elif mbtype == 'SRB':
             FW_mb = 0.230
      elif mbtype == 'TRB1':
             D_mb_a = 0.6604
             FW_mb = 0.1365
      elif mbtype == 'CRB':
             FW_mb = 0.136   
      elif mbtype == 'RB':
             FW_mb = 0.103      
      elif mbtype == 'TRB2':
             D_mb_a = 0.6858
             FW_mb = 0.200025

    elif D_mb <= 0.710:
      D_mb_a = 0.710
      if mbtype == 'CARB':
             FW_mb = 0.236
      elif mbtype == 'SRB':
             FW_mb = 0.236
      elif mbtype == 'TRB1':
             D_mb_a = .710
             FW_mb = 0.113
      elif mbtype == 'CRB':
             FW_mb = 0.140   
      elif mbtype == 'RB':
             FW_mb = 0.106      
      elif mbtype == 'TRB2':
             D_mb_a = 0.7112
             FW_mb = 0.1905

    elif D_mb <= 0.75:
      D_mb_a = 0.75
      if mbtype == 'CARB':
             FW_mb = 0.25
      elif mbtype == 'SRB':
             FW_mb = 0.25
      elif mbtype == 'TRB1':
             D_mb_a = .7493
             FW_mb = 0.1595
      elif mbtype == 'CRB':
             FW_mb = 0.112   
      elif mbtype == 'RB':
             FW_mb = 0.112      
      elif mbtype == 'TRB2':
             D_mb_a = 0.762
             FW_mb = 0.1873

    elif D_mb <= 0.80:
      D_mb_a = 0.80
      if mbtype == 'CARB':
             FW_mb = 0.258
      elif mbtype == 'SRB':
             FW_mb = 0.258
      elif mbtype == 'TRB1':
             D_mb_a = .801688
             FW_mb = 0.058738
      elif mbtype == 'CRB':
             FW_mb = 0.155   
      elif mbtype == 'RB':
             FW_mb = 0.115      
      elif mbtype == 'TRB2':
             D_mb_a = 0.8128
             FW_mb = 0.1905  
             
    elif D_mb <= 0.85:
      D_mb_a = 0.85
      if mbtype == 'CARB':
             FW_mb = 0.272
      elif mbtype == 'SRB':
             FW_mb = 0.272
      elif mbtype == 'TRB1':
             D_mb_a = .85725
             FW_mb = 0.12065
      elif mbtype == 'CRB':
             FW_mb = 0.118   
      elif mbtype == 'RB':
             FW_mb = 0.118      
      elif mbtype == 'TRB2':
             D_mb_a = 0.8636
             FW_mb = 0.4699                        

    elif D_mb <= 0.90:
      D_mb_a = 0.90
      if mbtype == 'CARB':
             FW_mb = 0.280
      elif mbtype == 'SRB':
             FW_mb = 0.280
      elif mbtype == 'TRB1':
             D_mb_a = .90
             FW_mb = 0.122
      elif mbtype == 'CRB':
             FW_mb = 0.122   
      elif mbtype == 'RB':
             FW_mb = 0.122      
      elif mbtype == 'TRB2':
             D_mb_a = 0.9144
             FW_mb = 0.1397

    elif D_mb <= 0.95:
      D_mb_a = 0.95
      if mbtype == 'CARB':
             FW_mb = 0.300
      elif mbtype == 'SRB':
             FW_mb = 0.300
      elif mbtype == 'TRB1':
             D_mb_a = .9779
             FW_mb = 0.06675
      elif mbtype == 'CRB':
             FW_mb = 0.175   
      elif mbtype == 'RB':
             FW_mb = 0.132      
      elif mbtype == 'TRB2':
             D_mb_a = 1.12
             FW_mb = 0.4

    elif D_mb <= 1.0:
      D_mb_a = 1.0
      if mbtype == 'CARB':
             FW_mb = 0.315
      elif mbtype == 'SRB':
             FW_mb = 0.315
      elif mbtype == 'TRB1':
             D_mb_a = 1.016
             FW_mb = 0.1016
      elif mbtype == 'CRB':
             FW_mb = 0.128   
      elif mbtype == 'RB':
             FW_mb = 0.140      
      elif mbtype == 'TRB2':
             D_mb_a = 1.12
             FW_mb = 0.4


    elif D_mb <= 1.06:
      D_mb_a = 1.06
      if mbtype == 'CARB':
             FW_mb = 0.25
      elif mbtype == 'SRB':
             FW_mb = 0.25
      elif mbtype == 'TRB1':
             D_mb_a = 1.270
             FW_mb = 0.10
      elif mbtype == 'CRB':
             FW_mb = 0.195   
      elif mbtype == 'RB':
             FW_mb = 0.150      
      elif mbtype == 'TRB2':
             D_mb_a = 1.12
             FW_mb = 0.4

    elif D_mb <= 1.18:
      D_mb_a = 1.18
      if mbtype == 'CARB':
             FW_mb = 0.272
      elif mbtype == 'SRB':
             FW_mb = 0.272
      elif mbtype == 'TRB1':
             D_mb_a = 1.270
             FW_mb = 0.10
      elif mbtype == 'CRB':
             FW_mb = 0.206   
      elif mbtype == 'RB':
             FW_mb = 0.160      
      elif mbtype == 'TRB2':
             D_mb_a = 1.25
             FW_mb = 0.25

    elif D_mb <= 1.25:
      D_mb_a = 1.25
      if mbtype == 'CARB':
             FW_mb = 0.375
      elif mbtype == 'SRB':
             FW_mb = 0.375
      elif mbtype == 'TRB1':
             D_mb_a = 1.270
             FW_mb = 0.10
      elif mbtype == 'CRB':
             FW_mb = 0.29   
      elif mbtype == 'RB':
             FW_mb = 0.112     
      elif mbtype == 'TRB2':
             D_mb_a = 1.25
             FW_mb = 0.25

    else:
      if mbtype == 'CARB':
             D_mb_a =1.25
             FW_mb = 0.375
      elif mbtype == 'SRB':
             D_mb_a =1.80      
             FW_mb = 0.375
      elif mbtype == 'TRB1':
             D_mb_a = 1.270
             FW_mb = 0.10
      elif mbtype == 'CRB':
             D_mb_a =1.40
             FW_mb = 0.175  
      elif mbtype == 'RB':
             D_mb_a =1.7      
             FW_mb = 0.212    
      elif mbtype == 'TRB2':
             D_mb_a = 1.778
             FW_mb = 0.3937             

    return [D_mb_a, FW_mb]

#-------------------------------------------------------------------------------
class LowSpeedShaft_drive4pt(Component):
    ''' LowSpeedShaft class
          The LowSpeedShaft class is used to represent the low speed shaft component of a wind turbine drivetrain. 
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.
    '''

    # variables
    rotor_bending_moment_x = Float(iotype='in', units='N*m', desc='The bending moment about the x axis')
    rotor_bending_moment_y = Float(iotype='in', units='N*m', desc='The bending moment about the y axis')
    rotor_bending_moment_z = Float(iotype='in', units='N*m', desc='The bending moment about the z axis')
    rotor_force_x = Float(iotype='in', units='N', desc='The force along the x axis applied at hub center')
    rotor_force_y = Float(iotype='in', units='N', desc='The force along the y axis applied at hub center')
    rotor_force_z = Float(iotype='in', units='N', desc='The force along the z axis applied at hub center')
    rotor_mass = Float(iotype='in', units='kg', desc='rotor mass')
    rotor_diameter = Float(iotype='in', units='m', desc='rotor diameter')
    machine_rating = Float(iotype='in', units='kW', desc='machine_rating machine rating of the turbine')
    gearbox_mass = Float(iotype='in', units='kg', desc='Gearbox mass')
    carrier_mass = Float(iotype='in', units='kg', desc='Carrier mass')
    overhang = Float(iotype='in', units='m', desc='Overhang distance')


    # parameters
    shrink_disc_mass = Float(iotype='in', units='kg', desc='Mass of the shrink disc')# shrink disk or flange addtional mass
    shaft_angle = Float(iotype='in', units='deg', desc='Angle of the LSS inclindation with respect to the horizontal')
    shaft_ratio = Float(iotype='in', desc='Ratio of inner diameter to outer diameter.  Leave zero for solid LSS')
    mb1Type = Str(iotype='in',desc='Main bearing type: CARB, TRB1 or SRB')
    mb2Type = Str(iotype='in',desc='Second bearing type: CARB, TRB1 or SRB')
    
    # outputs
    design_torque = Float(iotype='out', units='N*m', desc='lss design torque')
    design_bending_load = Float(iotype='out', units='N', desc='lss design bending load')
    length = Float(iotype='out', units='m', desc='lss length')
    diameter1 = Float(iotype='out', units='m', desc='lss outer diameter at main bearing')
    diameter2 = Float(iotype='out', units='m', desc='lss outer diameter at second bearing')
    mass = Float(0.0, iotype='out', units='kg', desc='overall component mass')
    cm = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system')
    I = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass')    

    def __init__(self):
        '''
        Initializes low speed shaft component

        Parameters
        ----------
        rotor_torque : float
          The torque load due to aerodynamic forces on the rotor [N*m]
        rotor_bending_moment : float
          The bending moment from uneven aerodynamic loads [N*m]
        rotor_mass : float
          The rotor mass [kg]
        rotor_diameter : float
          The wind turbine rotor diameter [m]
        rotor_speed : float
          The speed of the rotor at rated power [rpm]
        shaft_angle : float
          Angle of the LSS inclindation with respect to the horizontal [deg]
        shaft_length : float
          Length of the LSS [m]
        machine_rating : float
          machine_rating power rating for the turbine [W]
        shaft_ratio : float
          Ratio of inner diameter to outer diameter.  Leave zero for solid LSS. 

        Returns
        -------
        mass : float
          mass of the component [kg]
        cm : array of float
          center of mass of the component (relative to tower top center) [m, m, m]
        I : array of float
          principle moments of inertia for the component (Ixx, iyy, Izz) around its center of mass (relative to tower top center) [kg*m^2, kg*m^2, kg*m^2]   
        '''

        super(LowSpeedShaft_drive4pt, self).__init__()
    
    def execute(self):
        #Hub Forces
        F_r_x = self.rotor_force_x            #External F_x
        F_r_y = self.rotor_force_y                 #External F_y
        F_r_z = self.rotor_force_z                  #External F_z
        M_r_x = self.rotor_bending_moment_x
        M_r_y = self.rotor_bending_moment_y
        M_r_z = self.rotor_bending_moment_z

        #input parameters
        g=9.81
        gamma=self.shaft_angle #deg LSS angle wrt horizontal
        PSF=1

        
                
        # initialization for iterations    
        L_ms_new = 0.0
        L_ms_0=0.5 # main shaft length downwind of main bearing
        L_ms=L_ms_0
        tol=1e-4 
        check_limit = 1.0
        dL=0.05
        counter = 0
        N_count=50
        N_count_2=2
        len_pts=101
        D_max=1
        D_min=0.2
        sR = self.shaft_ratio

        #Distances
        L_rb = 1.912        #distance from hub center to main bearing   # to add as an input
        L_bg = 6.11         #distance from hub center to gearbox yokes  # to add as an input
        L_as = L_ms/2.0     #distance from main bearing to shaft center
        L_gb = 0.0          #distance to gbx center from trunnions in x-dir # to add as an input
        H_gb = 1.0          #distance to gbx center from trunnions in z-dir # to add as an input     
        L_gp = 0.825        #distance from gbx coupling to gbx trunnions
        L_cu = L_ms + 0.5   #distance from upwind main bearing to upwind carrier bearing 0.5 meter is an estimation # to add as an input
        L_cd = L_cu + 0.5   #distance from upwind main bearing to downwind carrier bearing 0.5 meter is an estimation # to add as an input
        
        #material properties
        E=2.1e11
        density=7800.0
        n_safety = 2.5 # According to AGMA, takes into account the peak load safety factor
        Sy = 66000 #psi

        #unit conversion
        u_knm_inlb = 8850.745454036
        u_in_m = 0.0254000508001

        #bearing deflection limits
        MB_limit = 0.026
        CB_limit = 4.0/60.0/180.0*pi
        TRB1_limit = 3.0/60.0/180.0*pi
        n_safety_brg = 1.0

        while abs(check_limit) > tol and L_ms_new < 0.5*self.overhang:
            counter = counter+1
            if L_ms_new > 0:
                L_ms=L_ms_new
            else:
                L_ms=L_ms_0

            #Distances
            L_as = L_ms/2.0     #distance from main bearing to shaft center
            L_cu = L_ms + 0.5   #distance from upwind main bearing to upwind carrier bearing 0.5 meter is an estimation # to add as an input
            L_cd = L_cu + 0.5   #distance from upwind main bearing to downwind carrier bearing 0.5 meter is an estimation # to add as an input

            #Weight properties
            rotorWeight=self.rotor_mass*g                             #rotor weight
            lssWeight = pi/3.0*(D_max**2 + D_min**2 + D_max*D_min)*L_ms*density*g/4.0 ##
            lss_mass = lssWeight/g
            gbxWeight = self.gearbox_mass*g                               #gearbox weight
            carrierWeight = self.carrier_mass*g                       #carrier weight
            shrinkDiscWeight = self.shrink_disc_mass*g

            #define LSS
            x_ms = np.linspace(L_rb, L_ms+L_rb, len_pts)
            x_rb = np.linspace(0.0, L_rb, len_pts)
            y_gp = np.linspace(0, L_gp, len_pts)

            F_mb_x = -F_r_x - rotorWeight*sin(radians(gamma))
            F_mb_y = +M_r_z/L_bg - F_r_y*(L_bg + L_rb)/L_bg
            F_mb_z = (-M_r_y + rotorWeight*(cos(radians(gamma))*(L_rb + L_bg)\
                       + sin(radians(gamma))*H_gb) + lssWeight*(L_bg - L_as)\
                       * cos(radians(gamma)) + shrinkDiscWeight*cos(radians(gamma))\
                       *(L_bg - L_ms) - gbxWeight*cos(radians(gamma))*L_gb - F_r_z*cos(radians(gamma))*(L_bg + L_rb))/L_bg

            F_gb_x = -(lssWeight+shrinkDiscWeight+gbxWeight)*sin(radians(gamma))
            F_gb_y = -F_mb_y - F_r_y
            F_gb_z = -F_mb_z + (shrinkDiscWeight+rotorWeight+gbxWeight + lssWeight)*cos(radians(gamma)) - F_r_z

            My_ms = np.zeros(2*len_pts)
            Mz_ms = np.zeros(2*len_pts)

            for k in range(len_pts):
                My_ms[k] = -M_r_y + rotorWeight*cos(radians(gamma))*x_rb[k] + 0.5*lssWeight/L_ms*x_rb[k]**2 - F_r_z*x_rb[k]
                Mz_ms[k] = -M_r_z - F_r_y*x_rb[k]

            for j in range(len_pts):
                My_ms[j+len_pts] = -F_r_z*x_ms[j] - M_r_y + rotorWeight*cos(radians(gamma))*x_ms[j] - F_mb_z*(x_ms[j]-L_rb) + 0.5*lssWeight/L_ms*x_ms[j]**2
                Mz_ms[j+len_pts] = -M_r_z - F_mb_y*(x_ms[j]-L_rb) -F_r_y*x_ms[j]

            x_shaft = np.concatenate([x_rb, x_ms])

            MM_max=np.amax((My_ms**2+Mz_ms**2)**0.5)
            Index=np.argmax((My_ms**2+Mz_ms**2)**0.5)

            MM_min = ((My_ms[-1]**2+Mz_ms[-1]**2)**0.5)
            #Design shaft OD 
            MM=MM_max
            D_max=(16.0*n_safety/pi/Sy*(4.0*(MM*u_knm_inlb/1000)**2+3.0*(M_r_x*u_knm_inlb/1000)**2)**0.5)**(1.0/3.0)*u_in_m

            #OD at end
            MM=MM_min
            D_min=(16.0*n_safety/pi/Sy*(4.0*(MM*u_knm_inlb/1000)**2+3.0*(M_r_x*u_knm_inlb/1000)**2)**0.5)**(1.0/3.0)*u_in_m

            #Estimate ID
            D_in=sR*D_max
            D_max = (D_max**4 + D_in**4)**0.25
            D_min = (D_min**4 + D_in**4)**0.25
           
            lssWeight_new=((pi/3)*(D_max**2+D_min**2+D_max*D_min)*(L_ms)*density/4+(-pi/4*(D_in**2)*density*(L_ms)))*g

            def deflection(F_r_z,W_r,gamma,M_r_y,f_mb_z,L_rb,W_ms,L_ms,z):
                return -F_r_z*z**3/6.0 + W_r*cos(radians(gamma))*z**3/6.0 - M_r_y*z**2/2.0 - f_mb_z*(z-L_rb)**3/6.0 + W_ms/(L_ms + L_rb)/24.0*z**4
            
                     
            D1 = deflection(F_r_z,rotorWeight,gamma,M_r_y,F_mb_z,L_rb,lssWeight_new,L_ms,L_rb+L_ms)
            D2 = deflection(F_r_z,rotorWeight,gamma,M_r_y,F_mb_z,L_rb,lssWeight_new,L_ms,L_rb)
            C1 = -(D1-D2)/L_ms;
            C2 = D2-C1*(L_rb);
            
            I_2=pi/64.0*(D_max**4 - D_in**4)

            def gx(F_r_z,W_r,gamma,M_r_y,f_mb_z,L_rb,W_ms,L_ms,C1,z):
                return -F_r_z*z**2/2.0 + W_r*cos(radians(gamma))*z**2/2.0 - M_r_y*z - f_mb_z*(z-L_rb)**2/2.0 + W_ms/(L_ms + L_rb)/6.0*z**3 + C1

            theta_y = np.zeros(len_pts)
            d_y = np.zeros(len_pts)

            for kk in range(len_pts):
                theta_y[kk]=gx(F_r_z,rotorWeight,gamma,M_r_y,F_mb_z,L_rb,lssWeight_new,L_ms,C1,x_ms[kk])/E/I_2
                d_y[kk]=(deflection(F_r_z,rotorWeight,gamma,M_r_y,F_mb_z,L_rb,lssWeight_new,L_ms,x_ms[kk])+C1*x_ms[kk]+C2)/E/I_2

            check_limit = abs(abs(theta_y[-1])-TRB1_limit/n_safety_brg)

            if check_limit < 0:
                L_ms_new = L_ms + dL

            else:
                L_ms_new = L_ms + dL

         #Initialization
        L_mb=L_ms_new
        counter_ms=0
        check_limit_ms=1.0
        L_mb_new=0.0
        L_mb_0=L_mb                     #main shaft length
        L_ms = L_ms_new
        dL_ms = 0.05
        dL = 0.0025

        while abs(check_limit_ms)>tol and L_mb_new < 0.5*self.overhang:
            counter_ms = counter_ms + 1
            if L_mb_new > 0:
                L_mb=L_mb_new
            else:
                L_mb=L_mb_0

            counter = 0.0
            check_limit=1.0
            L_ms_gb_new=0.0
            L_ms_0=0.5 #mainshaft length
            L_ms = L_ms_0

            while abs(check_limit) > tol and counter <N_count_2:
                counter = counter+1
                if L_ms_gb_new>0.0:
                    L_ms_gb = L_ms_gb_new
                else:
                    L_ms_gb = L_ms_0

                #Distances
                L_as = (L_ms_gb+L_mb)/2.0
                L_cu = (L_ms_gb + L_mb) + 0.5
                L_cd = L_cu + 0.5

                #Weight
                lssWeight_new=((pi/3)*(D_max**2+D_min**2+D_max*D_min)*(L_ms_gb + L_mb)*density/4+(-pi/4*(D_in**2)*density*(L_ms_gb + L_mb)))*g

                #define LSS
                x_ms = np.linspace(L_rb + L_mb, L_ms_gb + L_mb +L_rb, len_pts)
                x_mb = np.linspace(L_rb, L_mb+L_rb, len_pts)
                x_rb = np.linspace(0.0, L_rb, len_pts)
                y_gp = np.linspace(0, L_gp, len_pts)

                F_mb2_x = -F_r_x - rotorWeight*sin(radians(gamma))
                F_mb2_y = -M_r_z/L_mb + F_r_y*(L_rb)/L_mb
                F_mb2_z = (M_r_y - rotorWeight*cos(radians(gamma))*L_rb \
                          -lssWeight*L_as*cos(radians(gamma)) - shrinkDiscWeight*L_ms*cos(radians(gamma)) \
                           + gbxWeight*cos(radians(gamma))*L_gb + F_r_z*cos(radians(gamma))*L_rb)/L_mb

                F_mb1_x = 0.0
                F_mb1_y = -F_r_y - F_mb2_y
                F_mb1_z = (rotorWeight + lssWeight + shrinkDiscWeight)*cos(radians(gamma)) - F_r_z - F_mb2_z

                F_gb_x = -(lssWeight+shrinkDiscWeight+gbxWeight)*sin(radians(gamma))
                F_gb_y = -F_mb_y - F_r_y
                F_gb_z = -F_mb_z + (shrinkDiscWeight+rotorWeight+gbxWeight + lssWeight)*cos(radians(gamma)) - F_r_z

                My_ms = np.zeros(3*len_pts)
                Mz_ms = np.zeros(3*len_pts)

                for k in range(len_pts):
                    My_ms[k] = -M_r_y + rotorWeight*cos(radians(gamma))*x_rb[k] + 0.5*lssWeight/L_ms*x_rb[k]**2 - F_r_z*x_rb[k]
                    Mz_ms[k] = -M_r_z - F_r_y*x_rb[k]

                for j in range(len_pts):
                    My_ms[j+len_pts] = -F_r_z*x_mb[j] - M_r_y + rotorWeight*cos(radians(gamma))*x_mb[j] - F_mb1_z*(x_mb[j]-L_rb) + 0.5*lssWeight/L_ms*x_mb[j]**2
                    Mz_ms[j+len_pts] = -M_r_z - F_mb1_y*(x_mb[j]-L_rb) -F_r_y*x_mb[j]

                for l in range(len_pts):
                    My_ms[l + 2*len_pts] = -F_r_z*x_ms[l] - M_r_y + rotorWeight*cos(radians(gamma))*x_ms[l] - F_mb1_z*(x_ms[l]-L_rb) -F_mb2_z*(x_ms[l] - L_rb - L_mb) + 0.5*lssWeight/L_ms*x_ms[l]**2
                    Mz_ms[l + 2*len_pts] = -M_r_z - F_mb_y*(x_ms[l]-L_rb) -F_r_y*x_ms[l]

                x_shaft = np.concatenate([x_rb, x_mb, x_ms])

                MM_max=np.amax((My_ms**2+Mz_ms**2)**0.5)
                Index=np.argmax((My_ms**2+Mz_ms**2)**0.5)

                MM_min = ((My_ms[-1]**2+Mz_ms[-1]**2)**0.5)

                MM_med = ((My_ms[-1 - len_pts]**2 + Mz_ms[-1 - len_pts]**2)**0.5)

                #Design Shaft OD using static loading and distortion energy theory
                MM=MM_max
                D_max=(16.0*n_safety/pi/Sy*(4.0*(MM*u_knm_inlb/1000)**2+3.0*(M_r_x*u_knm_inlb/1000)**2)**0.5)**(1.0/3.0)*u_in_m

                #OD at end
                MM=MM_min
                D_min=(16.0*n_safety/pi/Sy*(4.0*(MM*u_knm_inlb/1000)**2+3.0*(M_r_x*u_knm_inlb/1000)**2)**0.5)**(1.0/3.0)*u_in_m

                MM=MM_med
                D_med=(16.0*n_safety/pi/Sy*(4.0*(MM*u_knm_inlb/1000)**2+3.0*(M_r_x*u_knm_inlb/1000)**2)**0.5)**(1.0/3.0)*u_in_m

                #Estimate ID
                D_in=sR*D_max
                D_max = (D_max**4 + D_in**4)**0.25
                D_min = (D_min**4 + D_in**4)**0.25
                D_med = (D_med**4 + D_in**4)**0.25

                lssWeight_new = (density*pi/12.0*L_mb*(D_max**2+D_med**2 + D_max*D_med) - density*pi/4.0*D_in**2*L_mb)*g

                #deflection between mb1 and mb2
                def deflection1(F_r_z,W_r,gamma,M_r_y,f_mb1_z,L_rb,W_ms,L_ms,L_mb,z):
                    return -F_r_z*z**3/6.0 + W_r*cos(radians(gamma))*z**3/6.0 - M_r_y*z**2/2.0 - f_mb1_z*(z-L_rb)**3/6.0 + W_ms/(L_ms + L_mb)/24.0*z**4
                
                D11 = deflection1(F_r_z,rotorWeight,gamma,M_r_y,F_mb1_z,L_rb,lssWeight_new,L_ms,L_mb,L_rb+L_mb)
                D21 = deflection1(F_r_z,rotorWeight,gamma,M_r_y,F_mb1_z,L_rb,lssWeight_new,L_ms,L_mb,L_rb)
                C11 = -(D11-D21)/L_mb
                C21 = -D21-C11*(L_rb)

                I_2=pi/64.0*(D_max**4 - D_in**4)

                def gx1(F_r_z,W_r,gamma,M_r_y,f_mb1_z,L_rb,W_ms,L_ms,L_mb,C11,z):
                    return -F_r_z*z**2/2.0 + W_r*cos(radians(gamma))*z**2/2.0 - M_r_y*z - f_mb1_z*(z - L_rb)**2/2.0 + W_ms/(L_ms + L_mb)/6.0*z**3 + C11

                theta_y = np.zeros(2*len_pts)
                d_y = np.zeros(2*len_pts)

                for kk in range(len_pts):
                    theta_y[kk]=gx1(F_r_z,rotorWeight,gamma,M_r_y,F_mb1_z,L_rb,lssWeight_new,L_ms,L_mb,C11,x_mb[kk])/E/I_2
                    d_y[kk]=(deflection1(F_r_z,rotorWeight,gamma,M_r_y,F_mb1_z,L_rb,lssWeight_new,L_ms,L_mb,x_mb[kk])+C11*x_mb[kk]+C21)/E/I_2

                #Deflection between mb2 and gbx
                def deflection2(F_r_z,W_r,gamma,M_r_y,f_mb1_z,f_mb2_z,L_rb,W_ms,L_ms,L_mb,z):
                    return -F_r_z*z**3/6.0 + W_r*cos(radians(gamma))*z**3/6.0 - M_r_y*z**2/2.0 - f_mb1_z*(z-L_rb)**3/6.0 + -f_mb2_z*(z - L_rb - L_mb)**3/6.0 + W_ms/(L_ms + L_mb)/24.0*z**4
            
                def gx2(F_r_z,W_r,gamma,M_r_y,f_mb1_z,f_mb2_z,L_rb,W_ms,L_ms,L_mb,z):
                    return -F_r_z*z**2/2.0 + W_r*cos(radians(gamma))*z**2/2.0 - M_r_y*z - f_mb1_z*(z - L_rb)**2/2.0 - f_mb2_z*(z - L_rb - L_mb)**2/2.0 + W_ms/(L_ms + L_mb)/6.0*z**3

                D12 = deflection2(F_r_z,rotorWeight,gamma,M_r_y,F_mb1_z,F_mb2_z,L_rb,lssWeight_new,L_ms,L_mb,L_rb+L_mb)
                D22 = gx2(F_r_z,rotorWeight,gamma,M_r_y,F_mb1_z,F_mb2_z,L_rb,lssWeight_new,L_ms,L_mb,L_rb+L_mb)
                C12 = gx1(F_r_z,rotorWeight,gamma,M_r_y,F_mb1_z,L_rb,lssWeight_new,L_ms,L_mb,C11,x_mb[-1])-D22
                C22 = -D12-C12*(L_rb + L_mb);

                for kk in range(len_pts):
                    theta_y[kk + len_pts]=(gx2(F_r_z,rotorWeight,gamma,M_r_y,F_mb1_z,F_mb2_z,L_rb,lssWeight_new,L_ms,L_mb,x_ms[kk]) + C12)/E/I_2
                    d_y[kk + len_pts]=(deflection2(F_r_z,rotorWeight,gamma,M_r_y,F_mb1_z,F_mb2_z,L_rb,lssWeight_new,L_ms,L_mb,x_ms[kk])+C12*x_ms[kk]+C22)/E/I_2

                check_limit = abs(abs(theta_y[-1])-TRB1_limit/n_safety_brg)

                if check_limit < 0:
                    L_ms__gb_new = L_ms_gb + dL
                else:
                    L_ms__gb_new = L_ms_gb + dL

                check_limit_ms = abs(abs(theta_y[-1]) - TRB1_limit/n_safety_brg)

                if check_limit_ms < 0:
                    L_mb_new = L_mb + dL_ms
                else:
                    L_mb_new = L_mb + dL_ms

        [D_max_a,FW_max] = resize_for_bearings(D_max,  self.mb1Type)        
        
        [D_med_a,FW_med] = resize_for_bearings(D_med,  self.mb2Type)   
            
        lss_mass_new=(pi/3)*(D_max_a**2+D_med_a**2+D_max_a*D_med_a)*(L_mb-(FW_max+FW_med)/2)*density/4+ \
                         (pi/4)*(D_max_a**2-D_in**2)*density*FW_max+\
                         (pi/4)*(D_med_a**2-D_in**2)*density*FW_med-\
                         (pi/4)*(D_in**2)*density*(L_mb+(FW_max+FW_med)/2)
        lss_mass_new *= 1.3 # add flange and shrink disk mass
        self.length=L_mb_new + (FW_max+FW_med)/2 # TODO: create linear relationship based on power rating
        print ("L_mb: {0}").format(L_mb)
        print ("LSS length, m: {0}").format(self.length)
        self.D_outer=D_max
        print ("Upwind MB OD, m: {0}").format(D_max_a)
        print ("Dnwind MB OD, m: {0}").format(D_med_a)
        print ("D_min: {0}").format(D_min)
        self.D_inner=D_in
        self.mass=lss_mass_new
        self.diameter1= D_max_a
        self.diameter2= D_med_a 

        # calculate mass properties
        cm = np.array([0.0,0.0,0.0])
        cm[0] = - (0.035 - 0.01) * self.rotor_diameter            # cm based on WindPACT work - halfway between locations of two main bearings
        cm[1] = 0.0
        cm[2] = 0.025 * self.rotor_diameter
        self.cm = cm

        I = np.array([0.0, 0.0, 0.0])
        I[0]  = self.mass * (self.D_inner ** 2.0 + self.D_outer ** 2.0) / 8.0
        I[1]  = self.mass * (self.D_inner ** 2.0 + self.D_outer ** 2.0 + (4.0 / 3.0) * (self.length ** 2.0)) / 16.0
        I[2]  = I[1]
        self.I = I

#-------------------------------------------------------------------------------
class LowSpeedShaft_drive3pt(Component):
    ''' LowSpeedShaft class
          The LowSpeedShaft class is used to represent the low speed shaft component of a wind turbine drivetrain. 
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.
    '''

    # variables
    rotor_bending_moment_x = Float(iotype='in', units='N*m', desc='The bending moment about the x axis')
    rotor_bending_moment_y = Float(iotype='in', units='N*m', desc='The bending moment about the y axis')
    rotor_bending_moment_z = Float(iotype='in', units='N*m', desc='The bending moment about the z axis')
    rotor_force_x = Float(iotype='in', units='N', desc='The force along the x axis applied at hub center')
    rotor_force_y = Float(iotype='in', units='N', desc='The force along the y axis applied at hub center')
    rotor_force_z = Float(iotype='in', units='N', desc='The force along the z axis applied at hub center')
    rotor_mass = Float(iotype='in', units='kg', desc='rotor mass')
    rotor_diameter = Float(iotype='in', units='m', desc='rotor diameter')
    machine_rating = Float(iotype='in', units='kW', desc='machine_rating machine rating of the turbine')
    gearbox_mass = Float(iotype='in', units='kg', desc='Gearbox mass')
    carrier_mass = Float(iotype='in', units='kg', desc='Carrier mass')

    # parameters
    shrink_disc_mass = Float(iotype='in', units='kg', desc='Mass of the shrink disc')
    shaft_angle = Float(iotype='in', units='deg', desc='Angle of the LSS inclindation with respect to the horizontal')
    shaft_ratio = Float(iotype='in', desc='Ratio of inner diameter to outer diameter.  Leave zero for solid LSS')
    mb1Type = Str(iotype='in',desc='Main bearing type: CARB, TRB1 or SRB')
    mb2Type = Str(iotype='in',desc='Second bearing type: CARB, TRB1 or SRB')    
    # outputs
    design_torque = Float(iotype='out', units='N*m', desc='lss design torque')
    design_bending_load = Float(iotype='out', units='N', desc='lss design bending load')
    length = Float(iotype='out', units='m', desc='lss length')
    diameter1 = Float(iotype='out', units='m', desc='lss outer diameter at main bearing')
    diameter2 = Float(iotype='out', units='m', desc='lss outer diameter at second bearing')
    mass = Float(0.0, iotype='out', units='kg', desc='overall component mass')
    cm = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system')
    I = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass')    

    def __init__(self):
        '''
        Initializes low speed shaft component

        Parameters
        ----------
        rotor_torque : float
          The torque load due to aerodynamic forces on the rotor [N*m]
        rotor_bending_moment : float
          The bending moment from uneven aerodynamic loads [N*m]
        rotor_mass : float
          The rotor mass [kg]
        rotor_diameter : float
          The wind turbine rotor diameter [m]
        rotor_speed : float
          The speed of the rotor at rated power [rpm]
        shaft_angle : float
          Angle of the LSS inclindation with respect to the horizontal [deg]
        shaft_length : float
          Length of the LSS [m]
        machine_rating : float
          machine_rating power rating for the turbine [W]
        shaft_ratio : float
          Ratio of inner diameter to outer diameter.  Leave zero for solid LSS. 

        Returns
        -------
        mass : float
          mass of the component [kg]
        cm : array of float
          center of mass of the component (relative to tower top center) [m, m, m]
        I : array of float
          principle moments of inertia for the component (Ixx, iyy, Izz) around its center of mass (relative to tower top center) [kg*m^2, kg*m^2, kg*m^2]   
        '''

        super(LowSpeedShaft_drive3pt, self).__init__()
    
    def execute(self):
        #Hub Forces
        F_r_x = self.rotor_force_x            #External F_x
        F_r_y = self.rotor_force_y                 #External F_y
        F_r_z = self.rotor_force_z                  #External F_z
        M_r_x = self.rotor_bending_moment_x
        M_r_y = self.rotor_bending_moment_y
        M_r_z = self.rotor_bending_moment_z
        
        g = 9.81 #m/s
        gamma = self.shaft_angle #deg LSS angle wrt horizontal
        PSF = 1.0
        density = 7850.0


        L_ms_new = 0.0
        L_ms_0=0.5 # main shaft length downwind of main bearing
        L_ms=L_ms_0
        tol=1e-4 
        check_limit = 1.0
        dL=0.05
        D_max = 1.0
        D_min = 0.2

        T=M_r_x/1000.0

        #Main bearing defelection check
        MB_limit=0.026;
        CB_limit=4.0/60.0/180.0*pi;
        TRB1_limit=3.0/60.0/180.0*pi;
        n_safety_brg = 1.0
        n_safety=2.5
        Sy = 66000.0 #psi
        E=2.1e11  
        N_count=50    
          
        u_knm_inlb = 8850.745454036
        u_in_m = 0.0254000508001
        counter=0
        while abs(check_limit) > tol and counter <N_count:
            counter = counter+1
            if L_ms_new > 0:
               	 L_ms=L_ms_new
            else:
                	L_ms=L_ms_0

            #Distances
            L_rb = 1.912 #*(self.machine_rating/5.0e3)   #distance from hub center to main bearing scaled off NREL 5MW
            L_bg = 6.11 #*(self.machine_rating/5.0e3)         #distance from hub center to gearbox yokes
            L_as = L_ms/2.0     #distance from main bearing to shaft center
            H_gb = 1.0          #distance to gbx center from trunnions in z-dir     
            L_gp = 0.825        #distance from gbx coupling to gbx trunnions
            L_cu = L_ms + 0.5
            L_cd = L_cu + 0.5
            L_gb=0

            #Weight properties
            weightRotor=0     # Yi modified to remove rotor overhung weight, considered in the load analysis                        #rotor weight accounted for in F_z
            massLSS = pi/3*(D_max**2.0 + D_min**2.0 + D_max*D_min)*L_ms*density/4.0
            weightLSS = massLSS*g       #LSS weight
            weightShrinkDisc = self.shrink_disc_mass*g                #shrink disc weight
            weightGbx = self.gearbox_mass*g                              #gearbox weight
            weightCarrier = self.carrier_mass*g

            len_pts=101;
            x_ms = np.linspace(L_rb, L_ms+L_rb, len_pts)
            x_rb = np.linspace(0.0, L_rb, len_pts)
            y_gp = np.linspace(0, L_gp, len_pts)

            #len_my = np.arange(1,len(M_r_y)+1)
            #print ("F_r_x: {0}").format(F_r_x)
            #print ("F_r_y: {0}").format(F_r_y)
            #print ("F_r_z: {0}").format(F_r_z)
            #print ("M_r_x: {0}").format(M_r_x)
            #print ("M_r_y: {0}").format(M_r_y)
            #print ("M_r_z: {0}").format(M_r_z)
            F_mb_x = -F_r_x - weightRotor*sin(radians(gamma))
            F_mb_y = M_r_z/L_bg - F_r_y*(L_bg + L_rb)/L_bg
            F_mb_z = (-M_r_y + weightRotor*(cos(radians(gamma))*(L_rb + L_bg)\
            + sin(radians(gamma))*H_gb) + weightLSS*(L_bg - L_as)\
            * cos(radians(gamma)) + weightShrinkDisc*cos(radians(gamma))\
            *(L_bg - L_ms) - weightGbx*cos(radians(gamma))*L_gb - F_r_z*cos(radians(gamma))*(L_bg + L_rb))/L_bg


            F_gb_x = -(weightLSS + weightShrinkDisc + weightGbx)*sin(radians(gamma))
            F_gb_y = -F_mb_y - F_r_y
            F_gb_z = -F_mb_z + (weightLSS + weightShrinkDisc + weightGbx + weightRotor)*cos(radians(gamma)) - F_r_z

            F_cu_z = (weightLSS*cos(radians(gamma)) + weightShrinkDisc*cos(radians(gamma)) + weightGbx*cos(radians(gamma))) - F_mb_z - F_r_z- \
            (-M_r_y - F_r_z*cos(radians(gamma))*L_rb + weightLSS*(L_bg - L_as)*cos(radians(gamma)) - weightCarrier*cos(radians(gamma))*L_gb)/(1 - L_cu/L_cd)

            F_cd_z = (weightLSS*cos(radians(gamma)) + weightShrinkDisc*cos(radians(gamma)) + weightGbx*cos(radians(gamma))) - F_mb_z - F_r_z - F_cu_z 


            My_ms = np.zeros(2*len_pts)
            Mz_ms = np.zeros(2*len_pts)

            for k in range(len_pts):
                My_ms[k] = -M_r_y + weightRotor*cos(radians(gamma))*x_rb[k] + 0.5*weightLSS/L_ms*x_rb[k]**2 - F_r_z*x_rb[k]
                Mz_ms[k] = -M_r_z - F_r_y*x_rb[k]

            for j in range(len_pts):
                My_ms[j+len_pts] = -F_r_z*x_ms[j] - M_r_y + weightRotor*cos(radians(gamma))*x_ms[j] - F_mb_z*(x_ms[j]-L_rb) + 0.5*weightLSS/L_ms*x_ms[j]**2
                Mz_ms[j+len_pts] = -M_r_z - F_mb_y*(x_ms[j]-L_rb) - F_r_y*x_ms[j]

            x_shaft = np.concatenate([x_rb, x_ms])

            MM_max=np.amax((My_ms**2 + Mz_ms**2)**0.5/1000.0)
            Index=np.argmax((My_ms**2 + Mz_ms**2)**0.5/1000.0)
                
            #print 'Max Moment kNm:'
            #print MM_max
            #print 'Max moment location m:'
            #print x_shaft[Index]

            MM_min = ((My_ms[-1]**2+Mz_ms[-1]**2)**0.5/1000.0)

            #print 'Max Moment kNm:'
            #print MM_min
            #print 'Max moment location m:'#
            #print x_shaft[-1]

            #Design shaft OD using distortion energy theory
            
           
            MM=MM_max
            D_max=(16.0*n_safety/pi/Sy*(4.0*(MM*u_knm_inlb)**2 + 3.0*(M_r_x/1000.0*u_knm_inlb)**2)**0.5)**(1.0/3.0)*u_in_m

            #OD at end
            MM=MM_min
            D_min=(16.0*n_safety/pi/Sy*(4.0*(MM*u_knm_inlb)**2 + 3.0*(M_r_x/1000.0*u_knm_inlb)**2)**0.5)**(1.0/3.0)*u_in_m

            #Estimate ID
            D_in=self.shaft_ratio*D_max
            D_max=(D_in**4.0 + D_max**4.0)**0.25
            D_min=(D_in**4.0 + D_min**4.0)**0.25
            #print'Max shaft OD m:'
            #print D_max
            #print 'Min shaft OD m:'
            #print D_min
            

            weightLSS_new = (density*pi/12.0*L_ms*(D_max**2.0 + D_min**2.0 + D_max*D_min) - density*pi/4.0*D_in**2.0*L_ms + \
            									density*pi/4.0*D_max**2*L_rb)*g
            massLSS_new = weightLSS_new/g

            #print 'Old LSS mass kg:' 
            #print massLSS
            #print 'New LSS mass kg:'
            #print massLSS_new

            def fx(F_r_z,W_r,gamma,M_r_y,f_mb_z,L_rb,W_ms,L_ms,z):
                return -F_r_z*z**3/6.0 + W_r*cos(radians(gamma))*z**3/6.0 - M_r_y*z**2/2.0 - f_mb_z*(z-L_rb)**3/6.0 + W_ms/(L_ms + L_rb)/24.0*z**4
            
                       
            D1 = fx(F_r_z,weightRotor,gamma,M_r_y,F_mb_z,L_rb,weightLSS_new,L_ms,L_rb+L_ms)
            D2 = fx(F_r_z,weightRotor,gamma,M_r_y,F_mb_z,L_rb,weightLSS_new,L_ms,L_rb)
            C1 = -(D1-D2)/L_ms;
            C2 = -D2-C1*(L_rb);
            
            
            I_2=pi/64.0*(D_max**4 - D_in**4)

            def gx(F_r_z,W_r,gamma,M_r_y,f_mb_z,L_rb,W_ms,L_ms,C1,z):
                return -F_r_z*z**2/2.0 + W_r*cos(radians(gamma))*z**2/2.0 - M_r_y*z - f_mb_z*(z-L_rb)**2/2.0 + W_ms/(L_ms + L_rb)/6.0*z**3 + C1

            theta_y = np.zeros(len_pts)
            d_y = np.zeros(len_pts)

            for kk in range(len_pts):
                theta_y[kk]=gx(F_r_z,weightRotor,gamma,M_r_y,F_mb_z,L_rb,weightLSS_new,L_ms,C1,x_ms[kk])/E/I_2
                d_y[kk]=(fx(F_r_z,weightRotor,gamma,M_r_y,F_mb_z,L_rb,weightLSS_new,L_ms,x_ms[kk])+C1*x_ms[kk]+C2)/E/I_2

            check_limit = abs(abs(theta_y[-1])-TRB1_limit/n_safety_brg)
            #print 'deflection slope'
            #print TRB1_limit
            #print 'threshold'
            #print theta_y[-1]
            L_ms_new = L_ms + dL        
            
            
        
        [D_max_a,FW_max] = resize_for_bearings(D_max,  self.mb1Type)        
        
        [D_min_a,FW_min] = resize_for_bearings(D_min,  self.mb2Type)   
            
        lss_mass_new=(pi/3)*(D_max_a**2+D_min_a**2+D_max_a*D_min_a)*(L_ms-(FW_max+FW_min)/2)*density/4+ \
                         (pi/4)*(D_max_a**2-D_in**2)*density*FW_max+\
                         (pi/4)*(D_min_a**2-D_in**2)*density*FW_min-\
                         (pi/4)*(D_in**2)*density*(L_ms+(FW_max+FW_min)/2)
        lss_mass_new *= 1.3 # add flange and shrink disk mass
        self.length=L_ms_new + (FW_max+FW_min)/2 # TODO: create linear relationship based on power rating
        #print ("L_ms: {0}").format(L_ms)
        #print ("LSS length, m: {0}").format(self.length)
        self.D_outer=D_max
        #print ("Upwind MB OD, m: {0}").format(D_max_a)
        #print ("CB OD, m: {0}").format(D_min_a)
        #print ("D_min: {0}").format(D_min)
        self.D_inner=D_in
        self.mass=lss_mass_new
        self.diameter1= D_max_a
        self.diameter2= D_min_a 
        #self.length=L_ms
        #print self.length
        self.D_outer=D_max_a
        self.diameter=D_max_a

        # calculate mass properties
        cm = np.array([0.0,0.0,0.0])
        cm[0] = - (0.035 - 0.01) * self.rotor_diameter            # cm based on WindPACT work - halfway between locations of two main bearings
        cm[1] = 0.0
        cm[2] = 0.025 * self.rotor_diameter
        self.cm = cm

        I = np.array([0.0, 0.0, 0.0])
        I[0]  = self.mass * (self.D_inner ** 2.0 + self.D_outer ** 2.0) / 8.0
        I[1]  = self.mass * (self.D_inner ** 2.0 + self.D_outer ** 2.0 + (4.0 / 3.0) * (self.length ** 2.0)) / 16.0
        I[2]  = I[1]
        self.I = I

#-------------------------------------------------------------------------------

class LowSpeedShaft_drive(Component):
    ''' LowSpeedShaft class
          The LowSpeedShaft class is used to represent the low speed shaft component of a wind turbine drivetrain. 
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.
    '''

    # variables
    rotor_torque = Float(iotype='in', units='N*m', desc='The torque load due to aerodynamic forces on the rotor')
    rotor_bending_moment = Float(iotype='in', units='N*m', desc='The bending moment from uneven aerodynamic loads')
    rotor_mass = Float(iotype='in', units='kg', desc='rotor mass')
    rotor_diameter = Float(iotype='in', units='m', desc='rotor diameter')
    rotor_speed = Float(iotype='in', units='rpm', desc='rotor speed at rated power')
    machine_rating = Float(iotype='in', units='kW', desc='machine_rating machine rating of the turbine')

    # parameters
    shaft_angle = Float(iotype='in', units='deg', desc='Angle of the LSS inclindation with respect to the horizontal')
    shaft_length = Float(iotype='in', units='m', desc='length of low speed shaft')
    shaftD1 = Float(iotype='in', units='m', desc='Fraction of LSS distance from gearbox to downwind main bearing')
    shaftD2 = Float(iotype='in', units='m', desc='raction of LSS distance from gearbox to upwind main bearing')
    shaft_ratio = Float(iotype='in', desc='Ratio of inner diameter to outer diameter.  Leave zero for solid LSS')
    
    # outputs
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
        rotor_torque : float
          The torque load due to aerodynamic forces on the rotor [N*m]
        rotor_bending_moment : float
          The bending moment from uneven aerodynamic loads [N*m]
        rotor_mass : float
          The rotor mass [kg]
        rotor_diameter : float
          The wind turbine rotor diameter [m]
        rotor_speed : float
          The speed of the rotor at rated power [rpm]
        shaft_angle : float
          Angle of the LSS inclindation with respect to the horizontal [deg]
        shaft_length : float
          Length of the LSS [m]
        shaftD1 : float
          Fraction of LSS distance from gearbox to downwind main bearing
        shaftD2 : float
          Fraction of LSS distance from gearbox to upwind main bearing
        machine_rating : float
          machine_rating power rating for the turbine [W]
        shaft_ratio : float
          Ratio of inner diameter to outer diameter.  Leave zero for solid LSS. 

        Returns
        -------
        mass : float
          mass of the component [kg]
        cm : array of float
          center of mass of the component (relative to tower top center) [m, m, m]
        I : array of float
          principle moments of inertia for the component (Ixx, iyy, Izz) around its center of mass (relative to tower top center) [kg*m^2, kg*m^2, kg*m^2]   
        '''

        super(LowSpeedShaft_drive, self).__init__()
    
    def execute(self):    

        def calc_mass(rotor_torque, rotor_bending_moment, rotor_mass, rotorDiaemeter, rotor_speed, shaft_angle, shaft_length, shaftD1, shaftD2, machine_rating, shaft_ratio):
        
                # Second moment of area for hollow shaft
            def Imoment(d_o,d_i):
                I=(pi/64.0)*(d_o**4-d_i**4)
                return I
            
            # Second polar moment for hollow shaft
            def Jmoment(d_o,d_i):
                J=(pi/32.0)*(d_o**4-d_i**4)
                return J
            
            # Bending stress
            def bendingStress(M, y, I):
                sigma=M*y/I
                return sigma
            
            # Shear stress
            def shearStress(T, r, J):
                tau=T*r/J
                return tau
            
            #Find the necessary outer diameter given a diameter ratio and max stress
            def outerDiameterStrength(shaft_ratio,maxFactoredStress):
                D_outer=(16.0/(pi*(1.0-shaft_ratio**4.0)*maxFactoredStress)*(factoredTotalRotorMoment+sqrt(factoredTotalRotorMoment**2.0+factoredrotor_torque**2.0)))**(1.0/3.0)
                return D_outer

            #[rotor_torque, rotor_bending_moment, rotor_mass, rotorDiaemeter, rotor_speed, shaft_angle, shaft_length, shaftD1, shaftD2, machine_rating, shaft_ratio] = x

            #torque check
            if rotor_torque == 0:
                omega=rotor_speed/60*(2*pi)      #rotational speed in rad/s at rated power
                eta=0.944                 #drivetrain efficiency
                rotor_torque=machine_rating/(omega*eta)         #torque

            #self.length=shaft_length
                
            # compute masses, dimensions and cost
            #static overhanging rotor moment (need to adjust for CM of rotor not just distance to end of LSS)
            L2=shaft_length*shaftD2                   #main bearing to end of mainshaft
            alpha=shaft_angle*pi/180.0           #shaft angle
            L2=L2*cos(alpha)                  #horizontal distance from main bearing to hub center of mass
            staticRotorMoment=rotor_mass*L2*9.81      #static bending moment from rotor
          
            #assuming 38CrMo4 / AISI 4140 from http://www.efunda.com/materials/alloys/alloy_steels/show_alloy.cfm?id=aisi_4140&prop=all&page_title=aisi%204140
            yieldStrength=417.0*10.0**6.0 #Pa
            steelDensity=8.0*10.0**3
            
            #Safety Factors
            gammaAero=1.35
            gammaGravity=1.35 #some talk of changing this to 1.1
            gammaFavorable=0.9
            gammaMaterial=1.25 #most conservative
            
            maxFactoredStress=yieldStrength/gammaMaterial
            factoredrotor_torque=rotor_torque*gammaAero
            factoredTotalRotorMoment=rotor_bending_moment*gammaAero-staticRotorMoment*gammaFavorable

            self.D_outer=outerDiameterStrength(self.shaft_ratio,maxFactoredStress)
            self.D_inner=shaft_ratio*self.D_outer

            #print "LSS outer diameter is %f m, inner diameter is %f m" %(self.D_outer, self.D_inner)
            
            J=Jmoment(self.D_outer,self.D_inner)
            I=Imoment(self.D_outer,self.D_inner)
            
            sigmaX=bendingStress(factoredTotalRotorMoment, self.D_outer/2.0, I)
            tau=shearStress(rotor_torque, self.D_outer/2.0, J)
            
            #print "Max unfactored normal bending stress is %g MPa" % (sigmaX/1.0e6)
            #print "Max unfactored shear stress is %g MPa" % (tau/1.0e6)
            
            volumeLSS=((self.D_outer/2.0)**2.0-(self.D_inner/2.0)**2.0)*pi*shaft_length
            mass=volumeLSS*steelDensity
            
            return mass
        
        self.mass = calc_mass(self.rotor_torque, self.rotor_bending_moment, self.rotor_mass, self.rotor_diameter, self.rotor_speed, \
                                    self.shaft_angle, self.shaft_length, self.shaftD1, self.shaftD2, self.machine_rating, self.shaft_ratio)
        

        self.design_torque = self.rotor_torque
        self.design_bending_load = self.rotor_bending_moment
        self.length = self.shaft_length
        self.diameter = self.D_outer

        # calculate mass properties
        cm = np.array([0.0,0.0,0.0])
        cm[0] = - (0.035 - 0.01) * self.rotor_diameter            # cm based on WindPACT work - halfway between locations of two main bearings
        cm[1] = 0.0
        cm[2] = 0.025 * self.rotor_diameter
        self.cm = cm

        I = np.array([0.0, 0.0, 0.0])
        I[0]  = self.mass * (self.D_inner ** 2.0 + self.D_outer ** 2.0) / 8.0
        I[1]  = self.mass * (self.D_inner ** 2.0 + self.D_outer ** 2.0 + (4.0 / 3.0) * (self.length ** 2.0)) / 16.0
        I[2]  = I[1]
        self.I = I

        '''# derivatives
        x = algopy.UTPM.init_jacobian([self.rotor_torque, self.rotor_bending_moment, self.rotor_mass, self.rotor_diameter, self.rotor_speed, self.machine_rating])  

        d_mass = algopy.UTPM.extract_jacobian(self.calc_mass(x))
        d_mass_d_rotor_torque = d_mass[0]
        d_mass_d_rotor_bending_moment = d_mass[1]
        d_mass_d_rotor_mass = d_mass[2]   
        d_mass_d_rotor_diameter = d_mass[3]
        d_mass_d_rotor_speed = d_mass[4]
        d_mass_d_machine_rating = d_mass[5] 

        d_cm_d_rotor_diameter = np.array([- (0.035 - 0.01), 0.0, 0.025])'''

        #TODO: d_I_d_x
        '''d_I_d_rotor_diameter = np.array([0.0, 0.0, 0.0])
        d_I_d_rotor_diameter[0] = (1/8) * (1+ ioratio**2) * d_lss_mass_d_rotor_diameter * (self.diameter**2) + (1/8) * (1+ioratio**2) * self.mass * 2 * self.diameter * d_diameter_d_rotor_diameter
        d_I_d_rotor_diameter[1] = (1/2) * d_I_d_rotor_diameter[0] + (1/16) * (4/3) * 2 * self.length * d_length_d_rotor_diameter
        d_I_d_rotor_diameter[2] = d_I_d_rotor_diameter[1]

        d_I_d_rotor_torque = np.array([0.0, 0.0, 0.0])
        d_I_d_rotor_torque[0] = (1/8) * (1+ ioratio**2) * d_lss_mass_d_rotor_torque * (self.diameter**2) + (1/8) * (1+ioratio**2) * self.mass * 2 * self.diameter * d_diameter_d_rotor_torque
        d_I_d_rotor_torque[1] = (1/2) * d_I_d_rotor_torque[0]
        d_I_d_rotor_torque[2] = d_I_d_rotor_torque[1]

        d_I_d_rotor_mass = np.array([0.0, 0.0, 0.0])
        d_I_d_rotor_mass[0] = (1/8) * (1+ ioratio**2) * d_lss_mass_d_rotor_mass * (self.diameter**2) + (1/8) * (1+ioratio**2) * self.mass * 2 * self.diameter * d_diameter_d_rotor_mass
        d_I_d_rotor_mass[1] = (1/2) * d_I_d_rotor_mass[0]
        d_I_d_rotor_mass[2] = d_I_d_rotor_mass[1] '''

    def provideJ(self):

        #TODO: provideJ update
        '''input_keys = ['rotor_diameter', 'rotor_torque', 'rotor_mass']
        output_keys = ['length', 'design_torque', 'design_bending_load', 'mass', 'cm[0]', 'cm[1]', 'cm[2]', 'I[0]', 'I[1]', 'I[2]']

        self.derivatives.set_first_derivative(input_keys, output_keys, self.J)'''


#-------------------------------------------------------------------------------

class MainBearings_drive(Component):
    ''' MainBearings class
          The MainBearings class is used to represent the main bearing components of a wind turbine drivetrain. It contains two subcomponents (main bearing and second bearing) which also inherit from the SubComponent class.
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.
    '''

    def __init__(self, shaftD1, shaftD2, shaft_length, rotor_mass, rotor_bending_moment, D_outer):
        ''' Initializes main bearings component

        Parameters
        ----------
        shaftD1 : float
          Fraction of LSS length from gearbox to downwind main bearing.
        shaftD2 : float
          Fraction of LSS length from gearbox to upwind main bearing.
        shaft_length : float
          Length of the LSS [m].
        rotor_mass : float
          Mass of the rotor [kg].
        rotor_bending_moment : float
          Aerodynamic bending moment [N*m].
        D_outer : float
          Outer diameter of the LSS [m].
        '''

        self.upwindBearing = Bearing()
        self.downwindBearing = Bearing()

        #compute reaction forces
        #Safety Factors
        gammaAero=1.35
        gammaGravity=1.35 #some talk of changing this to 1.1
        gammaFavorable=0.9
        gammaMaterial=1.25 #most conservative
        
        #Bearing 1 is closest to gearbox, Bearing 2 is closest to rotor
        L2=shaft_length*shaftD2
        L1=shaft_length*shaftD1

        Fstatic=rotor_mass*9.81*gammaFavorable #N
        Mrotor=rotor_bending_moment*gammaAero #Nm

        R2=(-Mrotor+Fstatic*shaftD1)/(shaftD2-shaftD1)
        #print "R2: %g" %(R2)

        R1=-Fstatic-R2
        #print "R1: %g" %(R1)     

        # compute masses, dimensions and cost

        self.upwindBearing.mass = 485.0
        self.downwindBearing.mass = 460.0

        self.mass = (self.upwindBearing.mass + self.downwindBearing.mass)

        # # calculate mass properties
        # inDiam  = lss.diameter
        # self.depth = (inDiam * 1.5)

        # cmMB = np.array([0.0,0.0,0.0])
        # cmMB = ([- (0.035 * rotor_diameter), 0.0, 0.025 * rotor_diameter])
        # self.mainBearing.cm = cmMB

        # cmSB = np.array([0.0,0.0,0.0])
        # cmSB = ([- (0.01 * rotor_diameter), 0.0, 0.025 * rotor_diameter])
        # self.secondBearing.cm = cmSB

        # cm = np.array([0.0,0.0,0.0])
        # for i in (range(0,3)):
        #     # calculate center of mass
        #     cm[i] = (self.mainBearing.mass * self.mainBearing.cm[i] + self.secondBearing.mass * self.secondBearing.cm[i]) \
        #               / (self.mainBearing.mass + self.secondBearing.mass)
        # self.cm = cm

        # self.b1I0 = (b1mass * inDiam ** 2 ) / 4 + (h1mass * self.depth ** 2) / 4
        # self.mainBearing.I = ([self.b1I0, self.b1I0 / 2, self.b1I0 / 2])

        # self.b2I0  = (b2mass * inDiam ** 2 ) / 4 + (h2mass * self.depth ** 2) / 4
        # self.secondBearing.I = ([self.b2I0, self.b2I0 / 2, self.b2I0 / 2])

        # I = np.array([0.0, 0.0, 0.0])
        # for i in (range(0,3)):                        # calculating MOI, at nacelle center of gravity with origin at tower top center / yaw mass center, ignoring masses of non-drivetrain components / auxiliary systems
        #     # calculate moments around CM
        #     # sum moments around each components CM
        #     I[i]  =  self.mainBearing.I[i] + self.secondBearing.I[i]
        #     # translate to nacelle CM using parallel axis theorem
        #     for j in (range(0,3)):
        #         if i != j:
        #             I[i] +=  (self.mainBearing.mass * (self.mainBearing.cm[i] - self.cm[i]) ** 2) + \
        #                           (self.secondBearing.mass * (self.secondBearing.cm[i] - self.cm[i]) ** 2)
        # self.I = I

#-------------------------------------------------------------------------------

class Bearing_drive(Component): 
    ''' MainBearings class          
          The MainBearings class is used to represent the main bearing components of a wind turbine drivetrain. It contains two subcomponents (main bearing and second bearing) which also inherit from the SubComponent class.
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.           
    '''
    # variables
    bearing_type = Str(iotype='in',desc='Main bearing type: CARB, TRB1 or SRB')
    lss_diameter = Float(iotype='in', units='m', desc='lss outer diameter at main bearing')
    lss_design_torque = Float(iotype='in', units='N*m', desc='lss design torque')
    rotor_diameter = Float(iotype='in', units='m', desc='rotor diameter')
    
    # returns
    mass = Float(0.0, iotype='out', units='kg', desc='overall component mass')
    cm = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system')
    I = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass')
    
    def __init__(self):
        
        super(Bearing_drive, self).__init__()
    
    def execute(self): #Modified by TParsons 4/21/14

        if self.lss_diameter <= 0.3:
            if self.bearing_type == 'CARB':
                self.mass = 120. #3250 kN
            elif self.bearing_type == 'SRB':
                self.mass = 128.7 #3070 kN
            elif self.bearing_type == 'TRB1':
                self.mass = 107.
            elif self.bearing_type == 'CRB':
                self.mass = 89.5
            elif self.bearing_type == 'RB':
                self.mass = 44
            elif self.bearing_type == 'TRB2':
                self.mass = 140
        if self.lss_diameter <= 0.34 :
            if self.bearing_type == 'CARB':
                self.mass = 205
            elif self.bearing_type == 'SRB':
                self.mass = 205
            elif self.bearing_type == 'TRB1':
                self.mass = 35
            elif self.bearing_type == 'CRB':
                self.mass = 65
            elif self.bearing_type == 'RB':
                self.mass = 62
            elif self.bearing_type == 'TRB2':
                self.mass = 71

        elif self.lss_diameter <= 0.38:
            if self.bearing_type == 'CARB':
                self.mass = 243
            elif self.bearing_type == 'SRB':
                self.mass = 300
            elif self.bearing_type == 'TRB1':
                self.mass = 20.5
            elif self.bearing_type == 'CRB':
                self.mass = 92.5
            elif self.bearing_type == 'RB':
                self.mass = 51
            elif self.bearing_type == 'TRB2':
                self.mass = 80.2

        elif self.lss_diameter <= 0.4:
            if self.bearing_type == 'CARB':
                self.mass = 258.
            elif self.bearing_type == 'SRB':
                self.mass = 265.
            elif self.bearing_type == 'TRB1':
                self.mass = 320
            elif self.bearing_type == 'CRB':
                self.mass = 120
            elif self.bearing_type == 'RB':
                self.mass = 27.5
            elif self.bearing_type == 'TRB2':
                self.mass = 82.4

        elif self.lss_diameter <= 0.44:
            if self.bearing_type == 'CARB':
                self.mass = 385.
            elif self.bearing_type == 'SRB':
                self.mass = 360.
            elif self.bearing_type == 'TRB1':
                self.mass = 120.
            elif self.bearing_type == 'CRB':
                self.mass = 145.
            elif self.bearing_type == 'RB':
                self.mass = 66.
            elif self.bearing_type == 'TRB2':
                self.mass = 203.

        elif self.lss_diameter <= 0.48:
            if self.bearing_type == 'CARB':
                self.mass = 523.
            elif self.bearing_type == 'SRB':
                self.mass = 470
            elif self.bearing_type == 'TRB1':
                self.mass = 190.
            elif self.bearing_type == 'CRB':
                self.mass = 170.
            elif self.bearing_type == 'RB':
                self.mass = 74.
            elif self.bearing_type == 'TRB2':
                self.mass = 302. 

        elif self.lss_diameter <= 0.5:
            if self.bearing_type == 'CARB':
                self.mass = 225.
            elif self.bearing_type == 'SRB':
                self.mass = 225.
            elif self.bearing_type == 'TRB1':
                self.mass = 59.5
            elif self.bearing_type == 'CRB':
                self.mass = 130.
            elif self.bearing_type == 'RB':
                self.mass = 77.
            elif self.bearing_type == 'TRB2':
                self.mass = 343.

        elif self.lss_diameter <= 0.56:
            if self.bearing_type == 'CARB':
                self.mass = 345.
            elif self.bearing_type == 'SRB':
                self.mass = 345.
            elif self.bearing_type == 'TRB1':
                self.mass = 115.
            elif self.bearing_type == 'CRB':
                self.mass = 145.
            elif self.bearing_type == 'RB':
                self.mass = 105.
            elif self.bearing_type == 'TRB2':
                self.mass = 150.

        elif self.lss_diameter <= 0.60:
            if self.bearing_type == 'CARB':
                self.mass = 390.
            elif self.bearing_type == 'SRB':
                self.mass = 405.
            elif self.bearing_type == 'TRB1':
                self.mass = 110.
            elif self.bearing_type == 'CRB':
                self.mass = 165.
            elif self.bearing_type == 'RB':
                self.mass = 125.
            elif self.bearing_type == 'TRB2':
                self.mass = 180.

        elif self.lss_diameter <= 0.67:
            if self.bearing_type == 'CARB':
                self.mass = 580.
            elif self.bearing_type == 'SRB':
                self.mass = 580.
            elif self.bearing_type == 'TRB1':
                self.mass = 287.
            elif self.bearing_type == 'CRB':
                self.mass = 360.
            elif self.bearing_type == 'RB':
                self.mass = 185.
            elif self.bearing_type == 'TRB2':
                self.mass = 270.                

        elif self.lss_diameter <= 0.710:
            if self.bearing_type == 'CARB':
                self.mass = 645.
            elif self.bearing_type == 'SRB':
                self.mass = 670.
            elif self.bearing_type == 'TRB1':
                self.mass = 200.
            elif self.bearing_type == 'CRB':
                self.mass = 300.
            elif self.bearing_type == 'RB':
                self.mass = 220.
            elif self.bearing_type == 'TRB2':
                self.mass = 265.

        elif self.lss_diameter <= 0.75:
            if self.bearing_type == 'CARB':
                self.mass = 838.
            elif self.bearing_type == 'SRB':
                self.mass = 770.
            elif self.bearing_type == 'TRB1':
                self.mass = 330.
            elif self.bearing_type == 'CRB':
                self.mass = 265.
            elif self.bearing_type == 'RB':
                self.mass = 225.
            elif self.bearing_type == 'TRB2':
                self.mass = 290.

        elif self.lss_diameter <= 0.80:
            if self.bearing_type == 'CARB':
                self.mass = 860.
            elif self.bearing_type == 'SRB':
                self.mass = 640.
            elif self.bearing_type == 'TRB1':
                self.mass = 53.5
            elif self.bearing_type == 'CRB':
                self.mass = 560.
            elif self.bearing_type == 'RB':
                self.mass = 320.
            elif self.bearing_type == 'TRB2':
                self.mass = 350.

        elif self.lss_diameter <= 0.85:
            if self.bearing_type == 'CARB':
                self.mass = 1105
            elif self.bearing_type == 'SRB':
                self.mass = 1050
            elif self.bearing_type == 'TRB1':
                self.mass = 245
            elif self.bearing_type == 'CRB':
                self.mass = 335.
            elif self.bearing_type == 'RB':
                self.mass = 310.
            elif self.bearing_type == 'TRB2':
                self.mass = 2250. #note: seems extremely high but is only TRB2 of this diameter on SKF site. Similar problems from here on

        elif self.lss_diameter <= 0.90:
            if self.bearing_type == 'CARB':
                self.mass = 1200.
            elif self.bearing_type == 'SRB':
                self.mass = 1200.
            elif self.bearing_type == 'TRB1':
                self.mass = 340.
            elif self.bearing_type == 'CRB':
                self.mass = 380.
            elif self.bearing_type == 'RB':
                self.mass = 350.
            elif self.bearing_type == 'TRB2':
                self.mass = 190.

        elif self.lss_diameter <= 0.95:
            if self.bearing_type == 'CARB':
                self.mass = 1410.
            elif self.bearing_type == 'SRB':
                self.mass = 1450.
            elif self.bearing_type == 'TRB1':
                self.mass = 100.
            elif self.bearing_type == 'CRB':
                self.mass = 745.
            elif self.bearing_type == 'RB':
                self.mass = 390.
            elif self.bearing_type == 'TRB2':
                self.mass = 1760.

        elif self.lss_diameter <= 1.0:
            if self.bearing_type == 'CARB':
                self.mass = 1570.
            elif self.bearing_type == 'SRB':
                self.mass = 2250.
            elif self.bearing_type == 'TRB1':
                self.mass = 275.
            elif self.bearing_type == 'CRB':
                self.mass = 350.
            elif self.bearing_type == 'RB':
                self.mass = 515.
            elif self.bearing_type == 'TRB2':
                self.mass = 1760.

        elif self.lss_diameter <= 1.06:
            if self.bearing_type == 'CARB':
                self.mass = 1120.
            elif self.bearing_type == 'SRB':
                self.mass = 1100.
            elif self.bearing_type == 'TRB1':
                self.mass = 265.
            elif self.bearing_type == 'CRB':
                self.mass = 870.
            elif self.bearing_type == 'RB':
                self.mass = 620.
            elif self.bearing_type == 'TRB2':
                self.mass = 1760.

        elif self.lss_diameter <= 1.18:
            if self.bearing_type == 'CARB':
                self.mass = 1400.
            elif self.bearing_type == 'SRB':
                self.mass = 1400.
            elif self.bearing_type == 'TRB1':
                self.mass = 265.
            elif self.bearing_type == 'CRB':
                self.mass = 1090.
            elif self.bearing_type == 'RB':
                self.mass = 775.
            elif self.bearing_type == 'TRB2':
                self.mass = 795.

        elif self.lss_diameter <= 1.25:
            if self.bearing_type == 'CARB':
                self.mass = 2740
            elif self.bearing_type == 'SRB':
                self.mass = 2840.
            elif self.bearing_type == 'TRB1':
                self.mass = 265.
            elif self.bearing_type == 'CRB':
                self.mass = 2320.
            elif self.bearing_type == 'RB':
                self.mass = 385.
            elif self.bearing_type == 'TRB2':
                self.mass = 795.

        else:
            if self.bearing_type == 'CARB':
                self.mass = 2740.
            elif self.bearing_type == 'SRB':
                self.mass = 2850.
            elif self.bearing_type == 'TRB1':
                self.mass = 265.
            elif self.bearing_type == 'CRB':
                self.mass = 860.
            elif self.bearing_type == 'RB':
                self.mass = 1950.
            elif self.bearing_type == 'TRB2':
                self.mass = 2750.                

        print self.mass
        self.mass += self.mass*(8000.0/2700.0) # add housing weight
        print ("Bearing Mass, kg: {0}").format(self.mass)

class MainBearing_drive(Bearing_drive): 
    ''' MainBearings class          
          The MainBearings class is used to represent the main bearing components of a wind turbine drivetrain. It contains two subcomponents (main bearing and second bearing) which also inherit from the SubComponent class.
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.           
    '''
    
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
        
        super(MainBearing_drive, self).__init__()
    
    def execute(self):

        super(MainBearing_drive, self).execute()
        
        # calculate mass properties
        inDiam  = self.lss_diameter
        depth = (inDiam * 1.5)

        cmMB = np.array([0.0,0.0,0.0])
        cmMB = ([- (0.035 * self.rotor_diameter), 0.0, 0.025 * self.rotor_diameter])
        self.cm = cmMB
       
        b1I0 = (self.mass * inDiam ** 2 ) / 4.0 
        self.I = ([b1I0, b1I0 / 2.0, b1I0 / 2.0])

    #     # derivatives
    #     if design1DL < ratingDL :
    #         d_mass_d_lssDiameter = 2.77 * 1000.0 * (massFact * (26.13 * (10 ** (-6))) * ((self.lssDiameter * 1000.0) ** 1.77)) + \
    #                                2.64 * 1000.0 * (massFact * (67.44 * (10 ** (-6))) * ((self.lssDiameter * 1000.0) ** 1.64))
    #     else :
    #         d_mass_d_lssDiameter = 2.77 * 1000.0 * (1.7 * massFact * (26.13 * (10 ** (-6))) * ((self.lssDiameter * 1000.0) ** 1.77)) + \
    #                                2.64 * 1000.0 * (1.5 * massFact * (67.44 * (10 ** (-6))) * ((self.lssDiameter * 1000.0) ** 1.64))
        
    #     d_cm_d_rotor_diameter = np.array([-0.035, 0.0, 0.025])
    #     d_I_d_lssDiameter = np.array([0.0, 0.0, 0.0])
    #     if design1DL < ratingDL :
    #         d_I_d_lssDiameter[0] = 2.77 * 1000.0 * (massFact * (26.13 * (10 ** (-6))) * ((self.lssDiameter * 1000.0) ** 1.77)) * ((inDiam**2)/4) + self.mass * (2/4) * inDiam + \
    #                                2.64 * 1000.0 * (massFact * (67.44 * (10 ** (-6))) * ((self.lssDiameter * 1000.0) ** 1.64)) * ((depth**2)/4) + self.mass * (2/4) * 1.5 * depth
    #     else :
    #         d_I_d_lssDiameter[0] = 2.77 * 1000.0 * (1.7 * massFact * (26.13 * (10 ** (-6))) * ((self.lssDiameter * 1000.0) ** 1.77)) * ((inDiam**2)/4) + self.mass * (2/4) * inDiam + \
    #                                   2.64 * 1000.0 * (1.5 * massFact * (67.44 * (10 ** (-6))) * ((self.lssDiameter * 1000.0) ** 1.64)) * ((depth**2)/4) + self.mass * (2/4) * 1.5 * depth
    #     d_I_d_lssDiameter[1] = (1/2) * d_I_d_lssDiameter[0]
    #     d_I_d_lssDiameter[2] = (1/2) * d_I_d_lssDiameter[0]

                               
        
    #     # Jacobian
    #     self.J = np.array([[d_mass_d_lssDiameter, 0], \
    #                        [0, d_cm_d_rotor_diameter[0]], \
    #                        [0, d_cm_d_rotor_diameter[1]], \
    #                        [0, d_cm_d_rotor_diameter[2]], \
    #                        [d_I_d_lssDiameter[0], 0], \
    #                        [d_I_d_lssDiameter[1], 0], \
    #                        [d_I_d_lssDiameter[2], 0]])

    # def provideJ(self):

    #     input_keys = ['lssDiameter', 'rotor_diameter']
    #     output_keys = ['mass', 'cm[0]', 'cm[1]', 'cm[2]', 'I[0]', 'I[1]', 'I[2]']

    #     self.derivatives.set_first_derivative(input_keys, output_keys, self.J)

#-------------------------------------------------------------------------------

class SecondBearing_drive(Bearing_drive): 
    ''' MainBearings class          
          The MainBearings class is used to represent the main bearing components of a wind turbine drivetrain. It contains two subcomponents (main bearing and second bearing) which also inherit from the SubComponent class.
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.           
    '''
    
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
        
        super(SecondBearing_drive, self).__init__()
    
    def execute(self):

        super(SecondBearing_drive, self).execute()

        # calculate mass properties
        inDiam  = self.lss_diameter
        depth = (inDiam * 1.5)

        cmSB = np.array([0.0,0.0,0.0])
        cmSB = ([- (0.01 * self.rotor_diameter), 0.0, 0.025 * self.rotor_diameter])
        self.cm = cmSB

        b2I0  = (self.mass * inDiam ** 2 ) / 4.0 
        self.I = ([b2I0, b2I0 / 2.0, b2I0 / 2.0])

#     #     # derivatives
#     #     if design2DL < ratingDL :
#     #         d_mass_d_lssDiameter = 2.77 * 1000.0 * (massFact * (26.13 * (10 ** (-6))) * ((self.lssDiameter * 1000.0) ** 1.77)) + \
#     #                                2.64 * 1000.0 * (massFact * (67.44 * (10 ** (-6))) * ((self.lssDiameter * 1000.0) ** 1.64))
#     #     else :
#     #         d_mass_d_lssDiameter = 2.77 * 1000.0 * (1.7 * massFact * (26.13 * (10 ** (-6))) * ((self.lssDiameter * 1000.0) ** 1.77)) + \
#     #                                2.64 * 1000.0 * (1.5 * massFact * (67.44 * (10 ** (-6))) * ((self.lssDiameter * 1000.0) ** 1.64))
        
#     #     d_cm_d_rotor_diameter = np.array([-0.01, 0.0, 0.025])
#     #     d_I_d_lssDiameter = np.array([0.0, 0.0, 0.0])
#     #     if design2DL < ratingDL :
#     #         d_I_d_lssDiameter[0] = 2.77 * 1000.0 * (massFact * (26.13 * (10 ** (-6))) * ((self.lssDiameter * 1000.0) ** 1.77)) * ((inDiam**2)/4) + self.mass * (2/4) * inDiam + \
#     #                                2.64 * 1000.0 * (massFact * (67.44 * (10 ** (-6))) * ((self.lssDiameter * 1000.0) ** 1.64)) * ((depth**2)/4) + self.mass * (2/4) * 1.5 * depth
#     #     else :
#     #         d_I_d_lssDiameter[0] = 2.77 * 1000.0 * (1.7 * massFact * (26.13 * (10 ** (-6))) * ((self.lssDiameter * 1000.0) ** 1.77)) * ((inDiam**2)/4) + self.mass * (2/4) * inDiam + \
#     #                                   2.64 * 1000.0 * (1.5 * massFact * (67.44 * (10 ** (-6))) * ((self.lssDiameter * 1000.0) ** 1.64)) * ((depth**2)/4) + self.mass * (2/4) * 1.5 * depth
#     #     d_I_d_lssDiameter[1] = (1/2) * d_I_d_lssDiameter[0]
#     #     d_I_d_lssDiameter[2] = (1/2) * d_I_d_lssDiameter[0]                             
        
#     #     # Jacobian
#     #     self.J = np.array([[d_mass_d_lssDiameter, 0], \
#     #                        [0, d_cm_d_rotor_diameter[0]], \
#     #                        [0, d_cm_d_rotor_diameter[1]], \
#     #                        [0, d_cm_d_rotor_diameter[2]], \
#     #                        [d_I_d_lssDiameter[0], 0], \
#     #                        [d_I_d_lssDiameter[1], 0], \
#     #                        [d_I_d_lssDiameter[2], 0]])

#     # def provideJ(self):

#     #     input_keys = ['lssDiameter', 'rotor_diameter']
#     #     output_keys = ['mass', 'cm[0]', 'cm[1]', 'cm[2]', 'I[0]', 'I[1]', 'I[2]']

#     #     self.derivatives.set_first_derivative(input_keys, output_keys, self.J)

#-------------------------------------------------------------------------------


class Gearbox_drive(Component):
    ''' Gearbox class
          The Gearbox class is used to represent the gearbox component of a wind turbine drivetrain.
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.
    '''

    #variables
    #gbxPower = Float(iotype='in', units='kW', desc='gearbox rated power')
    gear_ratio = Float(iotype='in', desc='overall gearbox speedup ratio')
    #check on how to define array input in openmdao
    Np = Array(np.array([0.0,0.0,0.0,]), iotype='in', desc='number of planets in each stage')
    rotor_speed = Float(iotype='in', desc='rotor rpm at rated power')
    rotor_diameter = Float(iotype='in', desc='rotor diameter')
    rotor_torque = Float(iotype='in', units='N*m', desc='rotor torque at rated power')
    length = Float(iotype='out', units='m', desc='gearbox length')
    height = Float(iotype='out', units='m', desc='gearbox height')
    diameter = Float(iotype='out', units='m', desc='gearbox diameter')

    #parameters
    #name = Str(iotype='in', desc='gearbox name')
    gear_configuration = Str(iotype='in', desc='string that represents the configuration of the gearbox (stage number and types)')
    #eff = Float(iotype='in', desc='drivetrain efficiency')
    ratio_type = Str(iotype='in', desc='optimal or empirical stage ratios')
    shaft_type = Str(iotype='in', desc = 'normal or short shaft length')

    # outputs
    stage_masses = Array(np.array([0.0, 0.0, 0.0, 0.0]), iotype='out', units='kg', desc='individual gearbox stage masses')
    mass = Float(0.0, iotype='out', units='kg', desc='overall component mass')
    cm = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system')
    I = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass')    


    def __init__(self):
        '''
        Initializes gearbox component

        Parameters
        ----------
        name : str
          Name of the gearbox.
        power : float
          Rated power of the gearbox [kW].
        gear_ratio : float
          Overall gearbox speedup ratio.
        gear_configuration : str
          String describing configuration of each gear stage.  Use 'e' for epicyclic and 'p' for parallel, for example 'eep' would be epicyclic-epicyclic-parallel.  'eep_3' and 'eep_2 are also options that fix the final stage ratio at 3 or 2 respectively.
        Np : array
          Array describing the number of planets in each stage.  For example if gearConfig is 'eep' Np could be [3 3 1].
        rotor_speed : float
          Rotational speed of the LSS at rated power [rpm].
        eff : float
          Mechanical efficiency of the gearbox.
        ratio_type : str
          Describes how individual stage ratios will be calculated.  Can be 'empirical' which uses the Sunderland model, or 'optimal' which finds the stage ratios that minimize overall mass.
        shaft_type : str
          Describes the shaft type and applies a corresponding application factor.  Can be 'normal' or 'short'.
        rotor_torque : float
          rotor torque.
        '''
        super(Gearbox_drive,self).__init__()

    def execute(self):

        self.stageRatio=np.zeros([3,1])

        self.stageTorque = np.zeros([len(self.stageRatio),1]) #filled in when ebxWeightEst is called
        self.stageMass = np.zeros([len(self.stageRatio),1]) #filled in when ebxWeightEst is called
        self.stageType=self.stageTypeCalc(self.gear_configuration)
        #print self.gear_ratio
        #print self.Np
        #print self.ratio_type
        #print self.gear_configuration
        self.stageRatio=self.stageRatioCalc(self.gear_ratio,self.Np,self.ratio_type,self.gear_configuration)
        #print self.stageRatio

        m=self.gbxWeightEst(self.gear_configuration,self.gear_ratio,self.Np,self.ratio_type,self.shaft_type,self.rotor_torque)
        self.mass = float(m)
        self.stage_masses=self.stageMass
        # calculate mass properties
        cm0   = 0.0
        cm1   = cm0
        cm2   = 0.025 * self.rotor_diameter
        self.cm = np.array([cm0, cm1, cm2])

        self.length = (0.012 * self.rotor_diameter)
        self.height = (0.015 * self.rotor_diameter)
        self.diameter = (0.75 * self.height)

        I0 = self.mass * (self.diameter ** 2 ) / 8 + (self.mass / 2) * (self.height ** 2) / 8
        I1 = self.mass * (0.5 * (self.diameter ** 2) + (2 / 3) * (self.length ** 2) + 0.25 * (self.height ** 2)) / 8
        I2 = I1
        self.I = np.array([I0, I1, I2])

        '''def rotor_torque():
            tq = self.gbxPower*1000 / self.eff / (self.rotor_speed * (pi / 30.0))
            return tq
        '''
     
    def stageTypeCalc(self, config):
        temp=[]
        for character in config:
                if character == 'e':
                    temp.append(2)
                if character == 'p':
                    temp.append(1)
        return temp

    def stageMassCalc(self, indStageRatio,indNp,indStageType):

        '''
        Computes the mass of an individual gearbox stage.

        Parameters
        ----------
        indStageRatio : str
          Speedup ratio of the individual stage in question.
        indNp : int
          Number of planets for the individual stage.
        indStageType : int
          Type of gear.  Use '1' for parallel and '2' for epicyclic.
        '''

        #Application factor to include ring/housing/carrier weight
        Kr=0.4
        Kgamma=1.1

        if indNp == 3:
            Kgamma=1.1
        elif indNp == 4:
            Kgamma=1.1
        elif indNp == 5:
            Kgamma=1.35

        if indStageType == 1:
            indStageMass=1.0+indStageRatio+indStageRatio**2+(1.0/indStageRatio)

        elif indStageType == 2:
            sunRatio=0.5*indStageRatio - 1.0
            indStageMass=Kgamma*((1/indNp)+(1/(indNp*sunRatio))+sunRatio+sunRatio**2+Kr*((indStageRatio-1)**2)/indNp+Kr*((indStageRatio-1)**2)/(indNp*sunRatio))

        return indStageMass
        
    def gbxWeightEst(self, config,overallRatio,Np,ratio_type,shaft_type,torque):


        '''
        Computes the gearbox weight based on a surface durability criteria.
        '''

        ## Define Application Factors ##
        #Application factor for weight estimate
        Ka=0.6
        Kshaft=0.0
        Kfact=0.0

        #K factor for pitting analysis
        if self.rotor_torque < 200000.0:
            Kfact = 850.0
        elif self.rotor_torque < 700000.0:
            Kfact = 950.0
        else:
            Kfact = 1100.0

        #Unit conversion from Nm to inlb and vice-versa
        Kunit=8.029

        # Shaft length factor
        if self.shaft_type == 'normal':
            Kshaft = 1.0
        elif self.shaft_type == 'short':
            Kshaft = 1.25

        #Individual stage torques
        torqueTemp=self.rotor_torque
        for s in range(len(self.stageRatio)):
            #print torqueTemp
            #print self.stageRatio[s]
            self.stageTorque[s]=torqueTemp/self.stageRatio[s]
            torqueTemp=self.stageTorque[s]
            self.stageMass[s]=Kunit*Ka/Kfact*self.stageTorque[s]*self.stageMassCalc(self.stageRatio[s],self.Np[s],self.stageType[s])
        
        gbxWeight=(sum(self.stageMass))*Kshaft
        
        return gbxWeight

    def stageRatioCalc(self, overallRatio,Np,ratio_type,config):
        '''
        Calculates individual stage ratios using either empirical relationships from the Sunderland model or a SciPy constrained optimization routine.
        '''

        K_r=0
                    
        #Assumes we can model everything w/Sunderland model to estimate speed ratio
        if ratio_type == 'empirical':
            if config == 'p': 
                x=[overallRatio]
            if config == 'e':
                x=[overallRatio]
            elif config == 'pp':
                x=[overallRatio**0.5,overallRatio**0.5]
            elif config == 'ep':
                x=[overallRatio/2.5,2.5]
            elif config =='ee':
                x=[overallRatio**0.5,overallRatio**0.5]
            elif config == 'eep':
                x=[(overallRatio/3)**0.5,(overallRatio/3)**0.5,3]
            elif config == 'epp':
                x=[overallRatio**(1.0/3.0),overallRatio**(1.0/3.0),overallRatio**(1.0/3.0)]
            elif config == 'eee':
                x=[overallRatio**(1.0/3.0),overallRatio**(1.0/3.0),overallRatio**(1.0/3.0)]
            elif config == 'ppp':
                x=[overallRatio**(1.0/3.0),overallRatio**(1.0/3.0),overallRatio**(1.0/3.0)]
        
        elif ratio_type == 'optimal':
            x=np.zeros([3,1])

            if config == 'eep':
                x0=[overallRatio**(1.0/3.0),overallRatio**(1.0/3.0),overallRatio**(1.0/3.0)]
                B_1=Np[0]
                B_2=Np[1]
                K_r1=0
                K_r2=0 #2nd stage structure weight coefficient

                def volume(x):
                    return (1.0/(x[0]))*((1.0/B_1)+(1.0/(B_1*((x[0]/2.0)-1.0)))+(x[0]/2.0-1.0)+ \
                    (x[0]/2.0-1)**2+K_r1*((x[0]-1.0)**2)/B_1 + K_r1*((x[0]-1.0)**2)/(B_1*(x[0]/2.0-1.0))) + \
                    (1.0/(x[0]*x[1]))*((1.0/B_2)+(1/(B_2*((x[1]/2.0)-1.0)))+(x[1]/2.0-1.0)+(x[1]/2.0-1.0)**2.0+K_r2*((x[1]-1.0)**2.0)/B_2 + \
                     K_r2*((x[1]-1.0)**2.0)/(B_2*(x[1]/2.0-1.0))) + (1.0/(x[0]*x[1]*x[2]))*(1.0+(1.0/x[2])+x[2] + x[2]**2)                              
                
                def constr1(x,overallRatio):
                    return x[0]*x[1]*x[2]-overallRatio
        
                def constr2(x,overallRatio):
                    return overallRatio-x[0]*x[1]*x[2]

                x=opt.fmin_cobyla(volume, x0,[constr1,constr2],consargs=[overallRatio],rhoend=1e-7, iprint = 0)
        
            elif config == 'eep_3':
                #fixes last stage ratio at 3
                x0=[overallRatio**(1.0/3.0),overallRatio**(1.0/3.0),overallRatio**(1.0/3.0)]
                B_1=Np[0]
                B_2=Np[1]
                K_r1=0
                K_r2=0.8 #2nd stage structure weight coefficient

                def volume(x):
                    return (1.0/(x[0]))*((1.0/B_1)+(1.0/(B_1*((x[0]/2.0)-1.0)))+(x[0]/2.0-1.0)+(x[0]/2.0-1)**2+K_r1*((x[0]-1.0)**2)/B_1 + K_r1*((x[0]-1.0)**2)/(B_1*(x[0]/2.0-1.0))) + (1.0/(x[0]*x[1]))*((1.0/B_2)+(1/(B_2*((x[1]/2.0)-1.0)))+(x[1]/2.0-1.0)+(x[1]/2.0-1.0)**2.0+K_r2*((x[1]-1.0)**2.0)/B_2 + K_r2*((x[1]-1.0)**2.0)/(B_2*(x[1]/2.0-1.0))) + (1.0/(x[0]*x[1]*x[2]))*(1.0+(1.0/x[2])+x[2] + x[2]**2)                              
                
                def constr1(x,overallRatio):
                    return x[0]*x[1]*x[2]-overallRatio
        
                def constr2(x,overallRatio):
                    return overallRatio-x[0]*x[1]*x[2]
                
                def constr3(x,overallRatio):
                    return x[2]-3.0
                
                def constr4(x,overallRatio):
                    return 3.0-x[2]

                x=opt.fmin_cobyla(volume, x0,[constr1,constr2,constr3,constr4],consargs=[overallRatio],rhoend=1e-7,iprint=0)
            
            elif config == 'eep_2':
                #fixes final stage ratio at 2
                x0=[overallRatio**(1.0/3.0),overallRatio**(1.0/3.0),overallRatio**(1.0/3.0)]
                B_1=Np[0]
                B_2=Np[1]
                K_r1=0
                K_r2=1.6 #2nd stage structure weight coefficient

                def volume(x):
                    return (1.0/(x[0]))*((1.0/B_1)+(1.0/(B_1*((x[0]/2.0)-1.0)))+(x[0]/2.0-1.0)+(x[0]/2.0-1)**2+K_r1*((x[0]-1.0)**2)/B_1 + K_r1*((x[0]-1.0)**2)/(B_1*(x[0]/2.0-1.0))) + (1.0/(x[0]*x[1]))*((1.0/B_2)+(1/(B_2*((x[1]/2.0)-1.0)))+(x[1]/2.0-1.0)+(x[1]/2.0-1.0)**2.0+K_r2*((x[1]-1.0)**2.0)/B_2 + K_r2*((x[1]-1.0)**2.0)/(B_2*(x[1]/2.0-1.0))) + (1.0/(x[0]*x[1]*x[2]))*(1.0+(1.0/x[2])+x[2] + x[2]**2)                              
                
                def constr1(x,overallRatio):
                    return x[0]*x[1]*x[2]-overallRatio
        
                def constr2(x,overallRatio):
                    return overallRatio-x[0]*x[1]*x[2]

                x=opt.fmin_cobyla(volume, x0,[constr1,constr2],consargs=[overallRatio],rhoend=1e-7, iprint = 0)
            elif config == 'epp':
                #fixes last stage ratio at 3
                x0=[overallRatio**(1.0/3.0),overallRatio**(1.0/3.0),overallRatio**(1.0/3.0)]
                B_1=Np[0]
                B_2=Np[1]
                K_r=0
               
                def volume(x):
                    return (1.0/(x[0]))*((1.0/B_1)+(1.0/(B_1*((x[0]/2.0)-1.0)))+(x[0]/2.0-1.0)+(x[0]/2.0-1)**2+ \
                    K_r*((x[0]-1.0)**2)/B_1 + K_r*((x[0]-1.0)**2)/(B_1*(x[0]/2.0-1.0))) + \
                    (1.0/(x[0]*x[1]))*(1.0+(1.0/x[1])+x[1] + x[1]**2) \
                    + (1.0/(x[0]*x[1]*x[2]))*(1.0+(1.0/x[2])+x[2] + x[2]**2)                              
                
                def constr1(x,overallRatio):
                    return x[0]*x[1]*x[2]-overallRatio
        
                def constr2(x,overallRatio):
                    return overallRatio-x[0]*x[1]*x[2]
                
                x=opt.fmin_cobyla(volume, x0,[constr1,constr2],consargs=[overallRatio],rhoend=1e-7,iprint=0)
                
            else:  # what is this subroutine for?  Yi on 04/16/2014
                x0=[overallRatio**(1.0/3.0),overallRatio**(1.0/3.0),overallRatio**(1.0/3.0)]
                B_1=Np[0]
                K_r=0.0
                def volume(x):
                    return (1.0/(x[0]))*((1.0/B_1)+(1.0/(B_1*((x[0]/2.0)-1.0)))+(x[0]/2.0-1)+(x[0]/2.0-1.0)**2+K_r*((x[0]-1.0)**2)/B_1 + K_r*((x[0]-1)**2)/(B_1*(x[0]/2.0-1.0))) + (1.0/(x[0]*x[1]))*(1.0+(1.0/x[1])+x[1] + x[1]**2)+ (1.0/(x[0]*x[1]*x[2]))*(1.0+(1.0/x[2])+x[2] + x[2]**2)
                                  
                def constr1(x,overallRatio):
                   return x[0]*x[1]*x[2]-overallRatio
        
                def constr2(x,overallRatio):
                    return overallRatio-x[0]*x[1]*x[2]

                x=opt.fmin_cobyla(volume, x0,[constr1,constr2],consargs=[overallRatio],rhoend=1e-7, iprint = 0)
        else:
            x='fail'
                  
        return x
        
#---------------------------------------------------------------------------------------------------------------

class Bedplate_drive(Component):
    ''' Bedplate class
          The Bedplate class is used to represent the bedplate of a wind turbine drivetrain.
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.
    '''

    #variables
    gbx_length = Float(iotype = 'in', units = 'm', desc = 'gearbox length')
    hss_location = Float(iotype ='in', units = 'm', desc='HSS CM location')
    hss_mass = Float(iotype ='in', units = 'kg', desc='HSS mass')
    generator_location = Float(iotype ='in', units = 'm', desc='generator CM location')
    generator_mass = Float(iotype ='in', units = 'kg', desc='generator mass')
    lss_location = Float(iotype ='in', units = 'm', desc='LSS CM location')
    lss_mass = Float(iotype ='in', units = 'kg', desc='LSS mass')
    lss_length = Float(iotype = 'in', units = 'm', desc = 'LSS length')
    mb1_location = Float(iotype ='in', units = 'm', desc='Upwind main bearing CM location')
    mb1_mass = Float(iotype ='in', units = 'kg', desc='Upwind main bearing mass')
    mb2_location = Float(iotype ='in', units = 'm', desc='Downwind main bearing CM location')
    mb2_mass = Float(iotype ='in', units = 'kg', desc='Downwind main bearing mass')
    transformer_mass = Float(iotype ='in', units = 'kg', desc='Transformer mass')
    tower_top_diameter = Float(iotype ='in', units = 'm', desc='diameter of the top tower section at the yaw gear')
    shaft_length = Float(iotype = 'in', units = 'm', desc='LSS length')
    rotor_diameter = Float(iotype = 'in', units = 'm', desc='rotor diameter')
    machine_rating = Float(iotype='in', units='kW', desc='machine_rating machine rating of the turbine')
    rotor_mass = Float(iotype='in', units='kg', desc='rotor mass')
    rotor_bending_moment_y = Float(iotype='in', units='N*m', desc='The bending moment about the y axis')
    rotor_force_z = Float(iotype='in', units='N', desc='The force along the z axis applied at hub center')
    flange_length = Float(iotype='in', units='kg', desc='flange length')


    #parameters
    #check openmdao syntax for boolean
    uptower_transformer = Bool(iotype = 'in', desc = 'Boolean stating if transformer is uptower')

    #outputs
    mass = Float(0.0, iotype='out', units='kg', desc='overall component mass')
    cm = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system')
    I = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass')    
    length = Float(iotype='out', units='m', desc='length of bedplate')

    width = Float(iotype='out', units='m', desc='width of bedplate')

    def __init__(self):
        ''' Initializes bedplate component

        Parameters
        ----------
        tower_top_diameter : float
          Diameter of the top tower section at the nacelle flange [m].
        shaft_length : float
          Length of the LSS [m]
        rotor_diameter : float
          The wind turbine rotor diameter [m].
        uptower_transformer : int
          Determines if the transformer is uptower ('1') or downtower ('0').
        '''

        #Treat front cast iron main beam in similar manner to hub
        #Use 1/4 volume of annulus given by:
        #length is LSS length plus tower radius
        #diameter is 1.25x tower diameter
        #guess at initial thickness same as hub
        #towerTopDiam, shaft_length, rotor_diameter, uptower_transformer=0

        super(Bedplate_drive,self).__init__()

    def execute(self):
        #Model bedplate as 2 parallel I-beams with a rear steel frame and a front cast frame
        #Deflection constraints applied at each bedplate end
        #Stress constraint checked at root of front and rear bedplate sections

        g = 9.81
        E = 2.1e11
        density = 7800

                #rear component weights and locations
        transLoc = 3.0*self.generator_location
        self.transformer_mass = 2.4445*(self.machine_rating) + 1599.0
        convLoc = 2.0*self.generator_location
        convMass = 0.3*self.transformer_mass 

        rearTotalLength = 0.0

        if transLoc > 0:
          rearTotalLength = transLoc + 1.0
        else:
          rearTotalLength = self.generator_location + 1.0


        #component masses and locations
        mb1_location = abs(self.gbx_length/2.0) + abs(self.lss_length)
        mb2_location = abs(self.gbx_length/2.0)
        lss_location= abs(self.lss_location)

        frontTotalLength = mb1_location + self.flange_length

        #rotor weights and loads
        rotorLoc = frontTotalLength
        rotorFz=abs(self.rotor_force_z)
        rotorMy=abs(self.rotor_bending_moment_y)
        rotorLoc=frontTotalLength

        #initial I-beam dimensions
        tf = 0.01905
        tw = 0.0127
        h0 = 0.6096
        b0 = h0/2.0

        stressTol = 1e6
        deflTol = 2.0e-3 # todo: model SUPER sensitive to this parameter... modified to achieve agreement with 5 MW turbine for now

        rootStress = 250e6
        totalTipDefl = 1.0

        def midDeflection(totalLength,loadLength,load,E,I):
          defl = load*loadLength**2.0*(3.0*totalLength - loadLength)/(6.0*E*I)
          return defl

          #tip deflection for distributed load
        def distDeflection(totalLength,distWeight,E,I):
          defl = distWeight*totalLength**4.0/(8.0*E*I)
          return defl

        
        #Rear Steel Frame:
        counter = 0
        while rootStress - 40.0e6 >  stressTol or totalTipDefl - 0.00001 >  deflTol:
          counter += 1
          bi = (b0-tw)/2.0
          hi = h0-2.0*tf
          I = b0*h0**3/12.0 - 2*bi*hi**3/12.0
          A = b0*h0 - 2.0*bi*hi
          w=A*density
          #Tip Deflection for load not at end
          

          hssTipDefl = midDeflection(rearTotalLength,self.hss_location,self.hss_mass*g/2,E,I)
          genTipDefl = midDeflection(rearTotalLength,self.generator_location,self.generator_mass*g/2,E,I)
          convTipDefl = midDeflection(rearTotalLength,convLoc,convMass*g/2,E,I)
          transTipDefl = midDeflection(rearTotalLength,transLoc,self.transformer_mass*g/2,E,I)
          selfTipDefl = distDeflection(rearTotalLength,w*g,E,I)

          totalTipDefl = hssTipDefl + genTipDefl + +convTipDefl + transTipDefl +  selfTipDefl 
          
          #root stress
          totalBendingMoment=(self.hss_location*self.hss_mass + self.generator_location*self.generator_mass + convLoc*convMass + transLoc*self.transformer_mass + w*rearTotalLength**2/2.0)*g
          rootStress = totalBendingMoment*h0/2/I

          #mass
          steelVolume = A*rearTotalLength
          steelMass = steelVolume*density

          #2 parallel I beams
          totalSteelMass = 2.0*steelMass

          rearTotalTipDefl=totalTipDefl
          rearBendingStress=rootStress

          tf += 0.002 
          tw += 0.002
          b0 += 0.006
          h0 += 0.006

          rearCounter = counter

        #Front cast section:

        E=169e9 #EN-GJS-400-18-LT http://www.claasguss.de/html_e/pdf/THBl2_engl.pdf
        castDensity = 7100

        tf = 0.01905
        tw = 0.0127
        h0 = 0.6096
        b0 = h0/2.0


        rootStress = 250e6
        totalTipDefl = 1.0
        counter = 0

        while rootStress - 40.0e6 >  stressTol or totalTipDefl - 0.00001 >  deflTol:
          counter += 1
          bi = (b0-tw)/2.0
          hi = h0-2.0*tf
          I = b0*h0**3/12.0 - 2*bi*hi**3/12.0
          A = b0*h0 - 2.0*bi*hi
          w=A*castDensity
          #Tip Deflection for load not at end
          

          mb1TipDefl = midDeflection(frontTotalLength,mb1_location,self.mb1_mass*g/2.0,E,I)
          mb2TipDefl = midDeflection(frontTotalLength,mb2_location,self.mb2_mass*g/2.0,E,I)
          lssTipDefl = midDeflection(frontTotalLength,lss_location,self.lss_mass*g/2.0,E,I)
          rotorTipDefl = midDeflection(frontTotalLength,rotorLoc,self.rotor_mass*g/2.0,E,I)
          rotorFzTipDefl = midDeflection(frontTotalLength,rotorLoc,rotorFz/2.0,E,I)
          selfTipDefl = distDeflection(frontTotalLength,w*g,E,I)
          rotorMyTipDefl = rotorMy/2.0*frontTotalLength**2/(2.0*E*I)
          #rotorMyTipDefl = rotorMy*frontTotalLength**2/(2.0*E*I)

          totalTipDefl = mb1TipDefl + mb2TipDefl + lssTipDefl  + rotorTipDefl + selfTipDefl +rotorMyTipDefl + rotorFzTipDefl

          #root stress
          totalBendingMoment=(mb1_location*self.mb1_mass/2.0 + mb2_location*self.mb2_mass/2.0 + lss_location*self.lss_mass/2.0 + w*frontTotalLength**2/2.0 + rotorLoc*self.rotor_mass/2.0)*g + rotorLoc*rotorFz/2.0 +rotorMy/2.0
          #totalBendingMoment=(mb1_location*self.mb1_mass/2.0 + mb2_location*self.mb2_mass/2.0 + lss_location*self.lss_mass/2.0 + w*frontTotalLength**2/2.0 + rotorLoc*self.rotor_mass)*g + rotorLoc*rotorFz/2.0 +rotorMy/2.0
          rootStress = totalBendingMoment*h0/2/I

          #mass
          castVolume = A*frontTotalLength
          castMass = castVolume*castDensity

          #2 parallel I-beams
          totalCastMass = 2.0*castMass


          frontTotalTipDefl=totalTipDefl
          frontBendingStress=rootStress

          tf += 0.002 
          tw += 0.002
          b0 += 0.006
          h0 += 0.006

          frontCounter = counter


        
        print 'rotor mass'
        print self.rotor_mass

        print 'rotor bending moment_y'
        print self.rotor_bending_moment_y
    

    	print 'rotor fz'
    	print self.rotor_force_z 

        print 'steel rear bedplate length: '
        print rearTotalLength

        print 'cast front bedplate length: '
        print frontTotalLength

        print b0
        print h0

        print'rear bedplate tip deflection'
        print rearTotalTipDefl

        print'front bedplate tip deflection'
        print frontTotalTipDefl

        print 'bending stress [MPa] at root of rear bedplate:'
        print rearBendingStress/1.0e6

        print 'bending stress [MPa] at root of front bedplate:'
        print frontBendingStress/1.0e6

        print 'cast front bedplate bedplate mass [kg]:'
        print totalCastMass

        print 'rear steel bedplate mass [kg]:'
        print totalSteelMass

        print 'total bedplate mass:'
        print totalSteelMass+ totalCastMass
        


        #frame multiplier for front support
        front_frame_support_multiplier = 1.33 # based on solidworks estimate
        totalCastMass *= front_frame_support_multiplier
        self.mass = totalCastMass+ totalSteelMass
        
        self.length = frontTotalLength + rearTotalLength
        self.width = b0 + self.tower_top_diameter

        # calculate mass properties
        cm = np.array([0.0,0.0,0.0])
        cm[0] = cm[1] = 0.0
        cm[2] = 0.0122 * self.rotor_diameter                             # half distance from shaft to yaw axis
        self.cm = cm

        self.depth = (self.length / 2.0)

        I = np.array([0.0, 0.0, 0.0])
        I[0]  = self.mass * (self.width ** 2 + self.depth ** 2) / 8
        I[1]  = self.mass * (self.depth ** 2 + self.width ** 2 + (4/3) * self.length ** 2) / 16
        I[2]  = I[1]
        self.I = I
        
#---------------------------------------------------------------------------------------------------------------

class Bedplate_drive_old(Component):
    ''' Bedplate class
          The Bedplate class is used to represent the bedplate of a wind turbine drivetrain.
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.
    '''

    #variables
    tower_top_diameter = Float(iotype ='in', desc='diameter of the top tower section at the yaw gear')
    shaft_length = Float(iotype = 'in', desc='LSS length')
    rotor_diameter = Float(iotype = 'in', desc='rotor diameter')

    #parameters
    #check openmdao syntax for boolean
    uptower_transformer = Bool(iotype = 'in', desc = 'Boolean stating if transformer is uptower')

    #outputs
    mass = Float(0.0, iotype='out', units='kg', desc='overall component mass')
    cm = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system')
    I = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass')    
    length = Float(iotype='out', units='m', desc='length of bedplate')

    width = Float(iotype='out', units='m', desc='width of bedplate')

    def __init__(self):
        ''' Initializes bedplate component

        Parameters
        ----------
        tower_top_diameter : float
          Diameter of the top tower section at the nacelle flange [m].
        shaft_length : float
          Length of the LSS [m]
        rotor_diameter : float
          The wind turbine rotor diameter [m].
        uptower_transformer : int
          Determines if the transformer is uptower ('1') or downtower ('0').
        '''

        #Treat front cast iron main beam in similar manner to hub
        #Use 1/4 volume of annulus given by:
        #length is LSS length plus tower radius
        #diameter is 1.25x tower diameter
        #guess at initial thickness same as hub
        #towerTopDiam, shaft_length, rotor_diameter, uptower_transformer=0

        super(Bedplate_drive,self).__init__()

    def execute(self):
        castThickness=self.rotor_diameter/620
        castVolume=(self.shaft_length+self.tower_top_diameter/2)*pi*(self.tower_top_diameter*1.25)*0.25*castThickness
        castDensity=7.1*10.0**3 #kg/m^3
        castMass=castDensity*castVolume
        self.length=0.0
        self.width=1.2
        self.depth=0.66
        
        #These numbers based off V80, need to update
        steelVolume=0.0
        if self.uptower_transformer == True:
            self.length=1.5*self.shaft_length
            steelVolume=self.length*self.width*self.depth/4.0
        elif self.uptower_transformer == False:
            self.length=self.shaft_length
            steelVolume=self.length*self.width*self.depth/4.0
        steelDensity=7900 #kg/m**3
        steelMass=steelDensity*steelVolume

        self.mass = steelMass + castMass

        # calculate mass properties
        cm = np.array([0.0,0.0,0.0])
        cm[0] = cm[1] = 0.0
        cm[2] = 0.0122 * self.rotor_diameter                             # half distance from shaft to yaw axis
        self.cm = cm

        self.depth = (self.length / 2.0)

        I = np.array([0.0, 0.0, 0.0])
        I[0]  = self.mass * (self.width ** 2 + self.depth ** 2) / 8
        I[1]  = self.mass * (self.depth ** 2 + self.width ** 2 + (4/3) * self.length ** 2) / 16
        I[2]  = I[1]
        self.I = I


#-------------------------------------------------------------------------------

class YawSystem_drive(Component):
    ''' YawSystem class
          The YawSystem class is used to represent the yaw system of a wind turbine drivetrain.
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.
    '''
    #variables
    rotor_diameter = Float(iotype='in', units='m', desc='rotor diameter')
    rotor_thrust = Float(iotype='in', units='N', desc='maximum rotor thrust')
    tower_top_diameter = Float(iotype='in', units='m', desc='tower top diameter')
    above_yaw_mass = Float(iotype='in', units='kg', desc='above yaw mass')

    #parameters
    yaw_motors_number = Float(iotype='in', desc='number of yaw motors')

    #outputs
    mass = Float(0.0, iotype='out', units='kg', desc='overall component mass')
    cm = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system')
    I = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass')    


    def __init__(self):
        ''' Initializes yaw system

        Parameters
        ----------
        towerTopDiam : float
          Diameter of the tower top section [m]
        rotor_diameter : float
          Rotor Diameter [m].
        yaw_motors_number : int
          Number of yaw motors.
        '''
        super(YawSystem_drive, self).__init__()

    def execute(self):

        if self.yaw_motors_number == 0 :
          if self.rotor_diameter < 90.0 :
            self.yaw_motors_number = 4.0
          elif self.rotor_diameter < 120.0 :
            self.yaw_motors_number = 6.0
          else:
            self.yaw_motors_number = 8.0

        #assume friction plate surface width is 1/10 the diameter
        #assume friction plate thickness scales with rotor diameter
        frictionPlateVol=pi*self.tower_top_diameter*(self.tower_top_diameter*0.10)*(self.rotor_diameter/1000.0)
        steelDensity=8000.0
        frictionPlateMass=frictionPlateVol*steelDensity
        
        #Assume same yaw motors as Vestas V80 for now: Bonfiglioli 709T2M
        yawMotorMass=190.0
        
        totalYawMass=frictionPlateMass + (self.yaw_motors_number*yawMotorMass)
        self.mass= totalYawMass

        # calculate mass properties
        # yaw system assumed to be collocated to tower top center
        cm = np.array([0.0,0.0,0.0])
        self.cm = cm

        # assuming 0 MOI for yaw system (ie mass is nonrotating)
        I = np.array([0.0, 0.0, 0.0])
        self.I = I

if __name__ == '__main__':
     pass
