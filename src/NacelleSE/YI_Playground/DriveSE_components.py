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
    rotorDiameter = Float(iotype='in', units='m', desc='rotor diameter')
    bladeRootDiam = Float(iotype='in', units='m', desc='blade root diameter')

    
    # parameters
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
        rotorDiameter : float
          The wind turbine rotor diameter [m]
        bladeRootDiam : float
          The diameter of the blade root [m]
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

        #Model hub as a cyclinder with holes for blade root and nacelle flange.
        rCyl=self.rotorDiameter/62.0
        hCyl=rCyl*3
        castThickness = rCyl/10.0
        approxCylVol=2*pi*rCyl*castThickness*hCyl
        bladeRootVol=pi*(bladeRootDiam/2.0)**2*castThickness

        #assume nacelle flange opening is similar to blade root opening
        approxCylNetVol = approxCylVol - (1.0 + bladeNumber)*bladeRootVol
        castDensity = 7200.0 # kg/m^3
        self.mass=approxCylNetVol*castDensity

        # calculate mass properties
        self.diameter=2*rCyl
        self.thickness=castThickness
                    
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

# Resize shaft for bearing selection
def resize_for_bearings(D_mb, mbtype):
    # Internal function to resize shaft for bearings - for Yi to add content (using lookup table etc)   
    # To add bearing load capacity check later        
    '''D_mb1 = 1.25
      D_mb2 = 0.75
      FW_mb1=0.45
      FW_mb2=0.5
    '''

    if D_mb < 0.3:
        if mbtype == 'CARB':
              D_mb_a = 0.3 #m
              FW_mb = 0.2
        elif self.bearing_type == 'SRB':
              D_mb_a = 0.3 #m
              FW_mb = 0.2
    elif D_mb < 0.4:
      if mbtype == 'CARB':
              D_mb_a = 0.4 #m
              FW_mb = 0.2
      elif mbtype == 'SRB':
              D_mb_a =  0.4 
              FW_mb =0.25       
    elif D_mb < 0.5:
      if mbtype == 'CARB':
              D_mb_a = 0.5 #
              FW_mb = 0.325
      elif mbtype == 'SRB':
              D_mb_a = 0.5 #
              FW_mb = 0.325                            
    elif D_mb < 0.6:
      if mbtype == 'CARB':
              D_mb_a = 0.6 #
              FW_mb = 0.375
      elif mbtype == 'SRB':
              D_mb_a = 0.6 #
              FW_mb = 0.375
    elif D_mb < 0.75:
      if mbtype == 'CARB':
              D_mb_a = 0.71 #
              FW_mb = 0.345
      elif mbtype == 'SRB':
              D_mb_a = 0.75 #
              FW_mb = 0.44          
    elif D_mb < 0.8:
      if mbtype == 'CARB':
              D_mb_a = 0.8 #
              FW_mb = 0.375
      elif mbtype == 'SRB':
              D_mb_a = 0.8 #
              FW_mb = 0.475
    elif D_mb < 0.95:
      if mbtype == 'CARB':
              D_mb_a = 0.95 #
              FW_mb = 0.3
      elif mbtype == 'SRB':
              D_mb_a = 0.95 #
              FW_mb = 0.525           
    elif D_mb < 1.0:
      if mbtype == 'CARB':
              D_mb_a = 1.0 #
              FW_mb = 0.375
      elif mbtype == 'SRB':
              D_mb_a = 1 #
              FW_mb = 0.5
    else:
      if mbtype == 'CARB':
              D_mb_a = 1.25 #
              FW_mb = 0.45
      elif mbtype == 'SRB':
              D_mb_a = 1.25 #
              FW_mb = 0.5   
    
    return [D_mb_a, FW_mb]

#-------------------------------------------------------------------------------
class LowSpeedShaft_drive4pt(Component):
    ''' LowSpeedShaft class
          The LowSpeedShaft class is used to represent the low speed shaft component of a wind turbine drivetrain. 
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.
    '''

    # variables
    rotorBendingMoment_x = Float(iotype='in', units='N*m', desc='The bending moment about the x axis')
    rotorBendingMoment_y = Float(iotype='in', units='N*m', desc='The bending moment about the y axis')
    rotorBendingMoment_z = Float(iotype='in', units='N*m', desc='The bending moment about the z axis')
    rotorForce_x = Float(iotype='in', units='N', desc='The force along the x axis applied at hub center')
    rotorForce_y = Float(iotype='in', units='N', desc='The force along the y axis applied at hub center')
    rotorForce_z = Float(iotype='in', units='N', desc='The force along the z axis applied at hub center')
    rotorMass = Float(iotype='in', units='kg', desc='rotor mass')
    rotorDiameter = Float(iotype='in', units='m', desc='rotor diameter')
    machineRating = Float(iotype='in', units='kW', desc='machineRating machine rating of the turbine')
    gbxMass = Float(iotype='in', units='kg', desc='Gearbox mass')
    carrierMass = Float(iotype='in', units='kg', desc='Carrier mass')

    # parameters
    shrinkDiscMass = Float(iotype='in', units='kg', desc='Mass of the shrink disc')# shrink disk or flange addtional mass
    shaftAngle = Float(iotype='in', units='deg', desc='Angle of the LSS inclindation with respect to the horizontal')
    shaftRatio = Float(iotype='in', desc='Ratio of inner diameter to outer diameter.  Leave zero for solid LSS')
    mb1Type = Str(iotype='in',desc='Main bearing type: CARB, TRB or SRB')
    mb2Type = Str(iotype='in',desc='Second bearing type: CARB, TRB or SRB')
    
    # outputs
    designTorque = Float(iotype='out', units='N*m', desc='lss design torque')
    designBendingLoad = Float(iotype='out', units='N', desc='lss design bending load')
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
        rotorTorque : float
          The torque load due to aerodynamic forces on the rotor [N*m]
        rotorBendingMoment : float
          The bending moment from uneven aerodynamic loads [N*m]
        rotorMass : float
          The rotor mass [kg]
        rotorDiameter : float
          The wind turbine rotor diameter [m]
        rotorSpeed : float
          The speed of the rotor at rated power [rpm]
        shaftAngle : float
          Angle of the LSS inclindation with respect to the horizontal [deg]
        shaftLength : float
          Length of the LSS [m]
        machineRating : float
          machineRating power rating for the turbine [W]
        shaftRatio : float
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
        F_r_x = self.rotorForce_x            #External F_x
        F_r_y = self.rotorForce_y                 #External F_y
        F_r_z = self.rotorForce_z                  #External F_z
        M_r_x = self.rotorBendingMoment_x
        M_r_y = self.rotorBendingMoment_y
        M_r_z = self.rotorBendingMoment_z

        #input parameters
        g=9.81
        gamma=self.shaftAngle #deg LSS angle wrt horizontal
        PSF=1

        
                
        # initialization for iterations    
        L_ms_new = 0.0
        L_ms_0=0.5 # main shaft length downwind of main bearing
        L_ms=L_ms_0
        tol=1e-4 
        check_limit = 1.0
        dL=0.05
        counter = 0
        N_count=100
        N_count_2=2
        len_pts=101
        D_max=1
        D_min=0.2
        sR = self.shaftRatio

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
        TRB_limit = 3.0/60.0/180.0*pi
        n_safety_brg = 1.0

        while abs(check_limit) > tol and counter <N_count:
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
            rotorWeight=self.rotorMass*g                             #rotor weight
            lssWeight = pi/3.0*(D_max**2 + D_min**2 + D_max*D_min)*L_ms*density*g/4.0 ##
            lssMass = lssWeight/g
            gbxWeight = self.gbxMass*g                               #gearbox weight
            carrierWeight = self.carrierMass*g                       #carrier weight
            shrinkDiscWeight = self.shrinkDiscMass*g

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

            check_limit = abs(abs(theta_y[-1])-TRB_limit/n_safety_brg)

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

        while abs(check_limit_ms)>tol and counter_ms<N_count:
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

                check_limit = abs(abs(theta_y[-1])-TRB_limit/n_safety_brg)

                if check_limit < 0:
                    L_ms__gb_new = L_ms_gb + dL
                else:
                    L_ms__gb_new = L_ms_gb + dL

                check_limit_ms = abs(abs(theta_y[-1]) - TRB_limit/n_safety_brg)

                if check_limit_ms < 0:
                    L_mb_new = L_mb + dL_ms
                else:
                    L_mb_new = L_mb + dL_ms

        [D_max_a,FW_max] = resize_for_bearings(D_max,  self.mb1Type)        
        
        [D_med_a,FW_med] = resize_for_bearings(D_med,  self.mb2Type)   
            
        lssMass_new=(pi/3)*(D_max_a**2+D_med_a**2+D_max_a*D_med_a)*(L_mb-(FW_max+FW_med)/2)*density/4+ \
                         (pi/4)*(D_max_a**2-D_in**2)*density*FW_max+\
                         (pi/4)*(D_med_a**2-D_in**2)*density*FW_med-\
                         (pi/4)*(D_in**2)*density*(L_mb+(FW_max+FW_med)/2)
        lssMass_new *= 1.3 # add flange and shrink disk mass
        self.length=L_mb_new + (FW_max+FW_med)/2 # TODO: create linear relationship based on power rating
        #print ("L_mb: {0}").format(L_mb)
        print ("LSS length, m: {0}").format(self.length)
        self.D_outer=D_max
        print ("Upwind MB OD, m: {0}").format(D_max_a)
        print ("Dnwind MB OD, m: {0}").format(D_med_a)
       # print ("D_min: {0}").format(D_min)
        self.D_inner=D_in
        self.mass=lssMass_new
        self.diameter1= D_max_a
        self.diameter2= D_med_a 

        # calculate mass properties
        cm = np.array([0.0,0.0,0.0])
        cm[0] = - (0.035 - 0.01) * self.rotorDiameter            # cm based on WindPACT work - halfway between locations of two main bearings
        cm[1] = 0.0
        cm[2] = 0.025 * self.rotorDiameter
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
    rotorBendingMoment_x = Float(iotype='in', units='N*m', desc='The bending moment about the x axis')
    rotorBendingMoment_y = Float(iotype='in', units='N*m', desc='The bending moment about the y axis')
    rotorBendingMoment_z = Float(iotype='in', units='N*m', desc='The bending moment about the z axis')
    rotorForce_x = Float(iotype='in', units='N', desc='The force along the x axis applied at hub center')
    rotorForce_y = Float(iotype='in', units='N', desc='The force along the y axis applied at hub center')
    rotorForce_z = Float(iotype='in', units='N', desc='The force along the z axis applied at hub center')
    rotorMass = Float(iotype='in', units='kg', desc='rotor mass')
    rotorDiameter = Float(iotype='in', units='m', desc='rotor diameter')
    machineRating = Float(iotype='in', units='kW', desc='machineRating machine rating of the turbine')
    gbxMass = Float(iotype='in', units='kg', desc='Gearbox mass')
    carrierMass = Float(iotype='in', units='kg', desc='Carrier mass')

    # parameters
    shrinkDiscMass = Float(iotype='in', units='kg', desc='Mass of the shrink disc')
    shaftAngle = Float(iotype='in', units='deg', desc='Angle of the LSS inclindation with respect to the horizontal')
    shaftRatio = Float(iotype='in', desc='Ratio of inner diameter to outer diameter.  Leave zero for solid LSS')
    
    # outputs
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
        rotorTorque : float
          The torque load due to aerodynamic forces on the rotor [N*m]
        rotorBendingMoment : float
          The bending moment from uneven aerodynamic loads [N*m]
        rotorMass : float
          The rotor mass [kg]
        rotorDiameter : float
          The wind turbine rotor diameter [m]
        rotorSpeed : float
          The speed of the rotor at rated power [rpm]
        shaftAngle : float
          Angle of the LSS inclindation with respect to the horizontal [deg]
        shaftLength : float
          Length of the LSS [m]
        machineRating : float
          machineRating power rating for the turbine [W]
        shaftRatio : float
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
        F_r_x = self.rotorForce_x            #External F_x
        F_r_y = self.rotorForce_y                 #External F_y
        F_r_z = self.rotorForce_z                  #External F_z
        M_r_x = self.rotorBendingMoment_x
        M_r_y = self.rotorBendingMoment_y
        M_r_z = self.rotorBendingMoment_z
        g = 9.81 #m/s
        gamma = self.shaftAngle #deg LSS angle wrt horizontal

        L_ms_new = 0.0
        L_ms_0=1.732 # main shaft length downwind of main bearing
        L_ms=L_ms_0
        tol=1e-4 
        check_limit = 1.0
        dL=0.025

        T=M_r_x/1000.0

        #Main bearing defelection check
        MB_limit = 0.026
        CB_limit = 4/60/180*pi
        n_safety_brg = 1.0

        while abs(check_limit) > tol:
            if L_ms_new > 0:
                L_ms=L_ms_new
            else:
                L_ms=L_ms_0

            #Distances
            L_rb = 1.912/2.0    #distance from hub center to main bearing
            L_bg = 6.11         #distance from hub center to gearbox yokes
            L_as = L_ms/2.0     #distance from main bearing to shaft center
            L_gb = 0.0          #distance to gbx center from trunnions in x-dir
            H_gb = 1.0          #distance to gbx center from trunnions in z-dir     
            L_gp = 0.825        #distance from gbx coupling to gbx trunnions

            #print L_rb

            #Weight properties
            weightRotor=self.rotorMass*g                             #rotor weight
            massLSS = pi/4*(0.5**2 - 0.075**2)*L_ms*7800
            weightLSS = massLSS*g       #LSS weight
            weightShrinkDisc = self.shrinkDiscMass*g                #shrink disc weight
            weightGbx = self.gbxMass*g                              #gearbox weight

            len_pts=101;
            x_ms = np.linspace(L_rb, L_ms+L_rb, len_pts)
            x_rb = np.linspace(0.0, L_rb, len_pts)
            y_gp = np.linspace(0, L_gp, len_pts)

            #len_my = np.arange(1,len(M_r_y)+1)

            F_mb_x = -F_r_x - weightRotor*sin(radians(g))
            F_mb_y = M_r_z/L_bg
            F_mb_z = (-M_r_y + weightRotor*(cos(radians(gamma))*(L_rb + L_bg)\
             + sin(radians(gamma))*H_gb) + weightLSS*(L_bg - L_as)\
             * cos(radians(gamma)) + weightShrinkDisc*cos(radians(gamma))\
             *(L_bg - L_ms) - weightGbx*cos(radians(gamma))*L_gb)/L_bg

            F_gb_x = -(weightLSS+weightShrinkDisc+weightGbx)*sin(radians(gamma))
            F_gb_y = -F_mb_y
            F_gb_z = -F_mb_z + (weightLSS+weightShrinkDisc+weightGbx + weightRotor)*sin(radians(gamma))

            My_ms = np.zeros(2*len_pts)
            Mz_ms = np.zeros(2*len_pts)

            for k in range(len_pts):
                My_ms[k] = -M_r_y + self.rotorMass*cos(radians(gamma))*x_rb[k] + 0.5*weightLSS/L_ms*x_rb[k]**2
                Mz_ms[k] = -M_r_z

            for j in range(len_pts):
                My_ms[j+len_pts] = -M_r_y + weightRotor*cos(radians(gamma))*x_ms[j] - F_mb_z*(x_ms[j]-L_rb) + 0.5*weightLSS/L_ms*x_ms[j]**2
                Mz_ms[j+len_pts] = -M_r_z - F_mb_y*(x_ms[j]-L_rb)

            x_shaft = np.concatenate([x_rb, x_ms])

            MM_max=np.amax((My_ms**2+Mz_ms**2)**0.5/1000.0)
            Index=np.argmax((My_ms**2+Mz_ms**2)**0.5/1000.0)

            #print 'Max Moment kNm:'
            #print MM_max
            #print 'Max moment location m:'
            #print x_shaft[Index]

            MM_min = ((My_ms[-1]**2+Mz_ms[-1]**2)**0.5/1000.0)

            #print 'Max Moment kNm:'
            #print MM_min
            #print 'Max moment location m:'
            #print x_shaft[-1]

            #Design shaft OD using distortion energy theory
            
            n_safety=1.8
            Sy = 66000.0 #psi

            #OD at Main Bearing
            u_knm_inlb = 8850.745454036
            u_in_m = 0.0254000508001
            MM=MM_max
            D_max=(16.0*n_safety/pi/Sy*(4.0*(MM*u_knm_inlb)**2+3.0*(T*u_knm_inlb)**2)**0.5)**(1.0/3.0)*u_in_m

            #OD at end
            MM=MM_min
            D_min=(16.0*n_safety/pi/Sy*(4.0*(MM*u_knm_inlb)**2+3.0*(T*u_knm_inlb)**2)**0.5)**(1.0/3.0)*u_in_m

            #Estimate ID
            D_in=self.shaftRatio*D_max
            D_max = D_max+D_in
            D_min = D_min + D_in
            #print'Max shaft OD m:'
            #print D_max
            #print 'Min shaft OD m:'
            #print D_min

            density = 7800.0
            weightLSS_new = (density*pi/12.0*L_ms*(D_max**2+D_min**2 + D_max*D_min) - density*pi/4.0*D_in**2*L_ms + density*pi/4.0*D_max**2*L_rb)*g
            massLSS_new = weightLSS_new/g

            #print 'Old LSS mass kg:' 
            #print massLSS
            #print 'New LSS mass kg:'
            #print massLSS_new

            def deflection(F_r_z,W_r,gamma,M_r_y,f_mb_z,L_rb,W_ms,L_ms,z):
                return -F_r_z*z**3/6.0 + W_r*cos(radians(gamma))*z**3/6.0 - M_r_y*z**2/2.0 - f_mb_z*(z-L_rb)**3/6.0 + W_ms/(L_ms + L_rb)/24.0*z**4
            
            z=L_rb+L_ms
            
            D1 = deflection(F_r_z,weightRotor,gamma,M_r_y,F_mb_z,L_rb,weightLSS_new,L_ms,z)
            D2 = deflection(F_r_z,weightRotor,gamma,M_r_y,F_mb_z,L_rb,weightLSS_new,L_ms,L_rb)
            C1 = -(D1-D2)/L_ms;
            C2 = D2-C1*(L_rb);
            
            E=2.1e11
            I_2=pi/64.0*(D_max**4 - D_in**4)

            def gx(F_r_z,W_r,gamma,M_r_y,f_mb_z,L_rb,W_ms,L_ms,C1,z):
                return -F_r_z*z**2/2.0 + W_r*cos(radians(gamma))*z**2/2.0 - M_r_y*z - f_mb_z*(z-L_rb)**2/2.0 + W_ms/(L_ms + L_rb)/6.0*z**3 + C1

            theta_y = np.zeros(len_pts)
            d_y = np.zeros(len_pts)

            for kk in range(len_pts):
                theta_y[kk]=gx(F_r_z,weightRotor,gamma,M_r_y,F_mb_z,L_rb,weightLSS_new,L_ms,C1,x_ms[kk])/E/I_2
                d_y[kk]=(deflection(F_r_z,weightRotor,gamma,M_r_y,F_mb_z,L_rb,weightLSS_new,L_ms,x_ms[kk])+C1*x_ms[kk]+C2)/E/I_2

            check_limit = abs(theta_y[-1])-CB_limit/n_safety_brg

            if check_limit < 0:
                L_ms_new = L_ms + dL

            else:
                L_ms_new = L_ms + dL

            #print 'new shaft length m:'
            #print L_ms_new

        self.length=L_ms_new
        self.D_outer=D_max
        #print self.D_outer
        self.D_inner=D_in
        #print self.D_in
        self.mass=massLSS_new
        self.diameter=D_max

        # calculate mass properties
        cm = np.array([0.0,0.0,0.0])
        cm[0] = - (0.035 - 0.01) * self.rotorDiameter            # cm based on WindPACT work - halfway between locations of two main bearings
        cm[1] = 0.0
        cm[2] = 0.025 * self.rotorDiameter
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
    rotorTorque = Float(iotype='in', units='N*m', desc='The torque load due to aerodynamic forces on the rotor')
    rotorBendingMoment = Float(iotype='in', units='N*m', desc='The bending moment from uneven aerodynamic loads')
    rotorMass = Float(iotype='in', units='kg', desc='rotor mass')
    rotorDiameter = Float(iotype='in', units='m', desc='rotor diameter')
    rotorSpeed = Float(iotype='in', units='rpm', desc='rotor speed at rated power')
    machineRating = Float(iotype='in', units='kW', desc='machineRating machine rating of the turbine')

    # parameters
    shaftAngle = Float(iotype='in', units='deg', desc='Angle of the LSS inclindation with respect to the horizontal')
    shaftLength = Float(iotype='in', units='m', desc='length of low speed shaft')
    shaftD1 = Float(iotype='in', units='m', desc='Fraction of LSS distance from gearbox to downwind main bearing')
    shaftD2 = Float(iotype='in', units='m', desc='raction of LSS distance from gearbox to upwind main bearing')
    shaftRatio = Float(iotype='in', desc='Ratio of inner diameter to outer diameter.  Leave zero for solid LSS')
    
    # outputs
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
        rotorTorque : float
          The torque load due to aerodynamic forces on the rotor [N*m]
        rotorBendingMoment : float
          The bending moment from uneven aerodynamic loads [N*m]
        rotorMass : float
          The rotor mass [kg]
        rotorDiameter : float
          The wind turbine rotor diameter [m]
        rotorSpeed : float
          The speed of the rotor at rated power [rpm]
        shaftAngle : float
          Angle of the LSS inclindation with respect to the horizontal [deg]
        shaftLength : float
          Length of the LSS [m]
        shaftD1 : float
          Fraction of LSS distance from gearbox to downwind main bearing
        shaftD2 : float
          Fraction of LSS distance from gearbox to upwind main bearing
        machineRating : float
          machineRating power rating for the turbine [W]
        shaftRatio : float
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

        def calc_mass(rotorTorque, rotorBendingMoment, rotorMass, rotorDiaemeter, rotorSpeed, shaftAngle, shaftLength, shaftD1, shaftD2, machineRating, shaftRatio):
        
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
            def outerDiameterStrength(shaftRatio,maxFactoredStress):
                D_outer=(16.0/(pi*(1.0-shaftRatio**4.0)*maxFactoredStress)*(factoredTotalRotorMoment+sqrt(factoredTotalRotorMoment**2.0+factoredrotorTorque**2.0)))**(1.0/3.0)
                return D_outer

            #[rotorTorque, rotorBendingMoment, rotorMass, rotorDiaemeter, rotorSpeed, shaftAngle, shaftLength, shaftD1, shaftD2, machineRating, shaftRatio] = x

            #torque check
            if rotorTorque == 0:
                omega=rotorSpeed/60*(2*pi)      #rotational speed in rad/s at rated power
                eta=0.944                 #drivetrain efficiency
                rotorTorque=machineRating/(omega*eta)         #torque

            #self.length=shaftLength
                
            # compute masses, dimensions and cost
            #static overhanging rotor moment (need to adjust for CM of rotor not just distance to end of LSS)
            L2=shaftLength*shaftD2                   #main bearing to end of mainshaft
            alpha=shaftAngle*pi/180.0           #shaft angle
            L2=L2*cos(alpha)                  #horizontal distance from main bearing to hub center of mass
            staticRotorMoment=rotorMass*L2*9.81      #static bending moment from rotor
          
            #assuming 38CrMo4 / AISI 4140 from http://www.efunda.com/materials/alloys/alloy_steels/show_alloy.cfm?id=aisi_4140&prop=all&page_title=aisi%204140
            yieldStrength=417.0*10.0**6.0 #Pa
            steelDensity=8.0*10.0**3
            
            #Safety Factors
            gammaAero=1.35
            gammaGravity=1.35 #some talk of changing this to 1.1
            gammaFavorable=0.9
            gammaMaterial=1.25 #most conservative
            
            maxFactoredStress=yieldStrength/gammaMaterial
            factoredrotorTorque=rotorTorque*gammaAero
            factoredTotalRotorMoment=rotorBendingMoment*gammaAero-staticRotorMoment*gammaFavorable

            self.D_outer=outerDiameterStrength(self.shaftRatio,maxFactoredStress)
            self.D_inner=shaftRatio*self.D_outer

            #print "LSS outer diameter is %f m, inner diameter is %f m" %(self.D_outer, self.D_inner)
            
            J=Jmoment(self.D_outer,self.D_inner)
            I=Imoment(self.D_outer,self.D_inner)
            
            sigmaX=bendingStress(factoredTotalRotorMoment, self.D_outer/2.0, I)
            tau=shearStress(rotorTorque, self.D_outer/2.0, J)
            
            #print "Max unfactored normal bending stress is %g MPa" % (sigmaX/1.0e6)
            #print "Max unfactored shear stress is %g MPa" % (tau/1.0e6)
            
            volumeLSS=((self.D_outer/2.0)**2.0-(self.D_inner/2.0)**2.0)*pi*shaftLength
            mass=volumeLSS*steelDensity
            
            return mass
        
        self.mass = calc_mass(self.rotorTorque, self.rotorBendingMoment, self.rotorMass, self.rotorDiameter, self.rotorSpeed, \
                                    self.shaftAngle, self.shaftLength, self.shaftD1, self.shaftD2, self.machineRating, self.shaftRatio)
        

        self.designTorque = self.rotorTorque
        self.designBendingLoad = self.rotorBendingMoment
        self.length = self.shaftLength
        self.diameter = self.D_outer

        # calculate mass properties
        cm = np.array([0.0,0.0,0.0])
        cm[0] = - (0.035 - 0.01) * self.rotorDiameter            # cm based on WindPACT work - halfway between locations of two main bearings
        cm[1] = 0.0
        cm[2] = 0.025 * self.rotorDiameter
        self.cm = cm

        I = np.array([0.0, 0.0, 0.0])
        I[0]  = self.mass * (self.D_inner ** 2.0 + self.D_outer ** 2.0) / 8.0
        I[1]  = self.mass * (self.D_inner ** 2.0 + self.D_outer ** 2.0 + (4.0 / 3.0) * (self.length ** 2.0)) / 16.0
        I[2]  = I[1]
        self.I = I

        '''# derivatives
        x = algopy.UTPM.init_jacobian([self.rotorTorque, self.rotorBendingMoment, self.rotorMass, self.rotorDiameter, self.rotorSpeed, self.machineRating])  

        d_mass = algopy.UTPM.extract_jacobian(self.calc_mass(x))
        d_mass_d_rotorTorque = d_mass[0]
        d_mass_d_rotorBendingMoment = d_mass[1]
        d_mass_d_rotorMass = d_mass[2]   
        d_mass_d_rotorDiameter = d_mass[3]
        d_mass_d_rotorSpeed = d_mass[4]
        d_mass_d_machineRating = d_mass[5] 

        d_cm_d_rotorDiameter = np.array([- (0.035 - 0.01), 0.0, 0.025])'''

        #TODO: d_I_d_x
        '''d_I_d_rotorDiameter = np.array([0.0, 0.0, 0.0])
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
        d_I_d_rotorMass[2] = d_I_d_rotorMass[1] '''

    def provideJ(self):

        #TODO: provideJ update
        '''input_keys = ['rotorDiameter', 'rotorTorque', 'rotorMass']
        output_keys = ['length', 'designTorque', 'designBendingLoad', 'mass', 'cm[0]', 'cm[1]', 'cm[2]', 'I[0]', 'I[1]', 'I[2]']

        self.derivatives.set_first_derivative(input_keys, output_keys, self.J)'''


#-------------------------------------------------------------------------------

class MainBearings_drive(Component):
    ''' MainBearings class
          The MainBearings class is used to represent the main bearing components of a wind turbine drivetrain. It contains two subcomponents (main bearing and second bearing) which also inherit from the SubComponent class.
          It contains the general properties for a wind turbine component as well as additional design load and dimentional attributes as listed below.
          It contains an update method to determine the mass, mass properties, and dimensions of the component.
    '''

    def __init__(self, shaftD1, shaftD2, shaftLength, rotorMass, rotorBendingMoment, D_outer):
        ''' Initializes main bearings component

        Parameters
        ----------
        shaftD1 : float
          Fraction of LSS length from gearbox to downwind main bearing.
        shaftD2 : float
          Fraction of LSS length from gearbox to upwind main bearing.
        shaftLength : float
          Length of the LSS [m].
        rotorMass : float
          Mass of the rotor [kg].
        rotorBendingMoment : float
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
        L2=shaftLength*shaftD2
        L1=shaftLength*shaftD1

        Fstatic=rotorMass*9.81*gammaFavorable #N
        Mrotor=rotorBendingMoment*gammaAero #Nm

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
        # cmMB = ([- (0.035 * rotorDiameter), 0.0, 0.025 * rotorDiameter])
        # self.mainBearing.cm = cmMB

        # cmSB = np.array([0.0,0.0,0.0])
        # cmSB = ([- (0.01 * rotorDiameter), 0.0, 0.025 * rotorDiameter])
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
    bearing_type = Str(iotype='in',desc='Main bearing type: CARB, TRB or SRB')
    lss_diameter = Float(iotype='in', units='m', desc='lss outer diameter at main bearing')
    lss_design_torque = Float(iotype='in', units='N*m', desc='lss design torque')
    rotor_diameter = Float(iotype='in', units='m', desc='rotor diameter')
    
    # returns
    mass = Float(0.0, iotype='out', units='kg', desc='overall component mass')
    cm = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc='center of mass of the component in [x,y,z] for an arbitrary coordinate system')
    I = Array(np.array([0.0, 0.0, 0.0]), iotype='out', desc=' moments of Inertia for the component [Ixx, Iyy, Izz] around its center of mass')
    
    def __init__(self):
        
        super(Bearing_drive, self).__init__()
    
    def execute(self):

        if self.lss_diameter < 0.3:
            if self.bearing_type == 'CARB':
                self.mass = 120 #3250 kN
            elif self.bearing_type == 'SRB':
                self.mass = 128.7 #3070 kN
            elif self.bearing_type == 'TRB':
                self.mass = 128.48 #2940 KN

        elif self.lss_diameter < 0.4:
            if self.bearing_type == 'CARB':
                self.mass = 145 #3650 kN
            elif self.bearing_type == 'SRB':
                self.mass = 148.7 #3310 kN
            elif self.bearing_type == 'TRB':
                self.mass = 162.55 #3210 KN
        
        elif self.lss_diameter < 0.5:
            if self.bearing_type == 'CARB':
                self.mass = 225 #4250 kN
            elif self.bearing_type == 'SRB':
                self.mass = 220 #4290 kN
            elif self.bearing_type == 'TRB':
                self.mass = 220.08 #3460 KN

        elif self.lss_diameter < 0.6:
            if self.bearing_type == 'CARB':
                self.mass = 390 #6300 kN
            elif self.bearing_type == 'SRB':
                self.mass = 390 #6040 kN
            elif self.bearing_type == 'TRB':
                self.mass = 323.6 #5730 KN

        elif self.lss_diameter < 0.7:  # modified by Y.G. FOR TSS 
            if self.bearing_type == 'CARB':
                self.mass = 645 #8800 kN
            elif self.bearing_type == 'SRB':
                self.mass = 1400 #8370 kN 
            elif self.bearing_type == 'TRB':
                self.mass = 513.79 #8740 KN

        elif self.lss_diameter < 0.8:
            if self.bearing_type == 'CARB':
                self.mass = 860 #9150 kN
            elif self.bearing_type == 'SRB':
                self.mass = 875 #9780 kN
            elif self.bearing_type == 'TRB':
                self.mass = 660.60 #8520 KN

        elif self.lss_diameter < 0.9:
            if self.bearing_type == 'CARB':
                self.mass = 1200 #12700 kN
            elif self.bearing_type == 'SRB':
                self.mass = 1322 #12200 kN
            elif self.bearing_type == 'TRB':
                self.mass = 1361.12 #12600 KN

        elif self.lss_diameter < 1.0:   # modified by Y.G. FOR TSS 
            if self.bearing_type == 'CARB':
                self.mass = 1570 #13400 kN
            elif self.bearing_type == 'SRB':
                self.mass = 1400 #14500 kN
            elif self.bearing_type == 'TRB':
                self.mass = 1061.43 #7550 KN

        else:                              # modified by Y.G. FOR TSS 
            if self.bearing_type == 'CARB':
                self.mass = 2740 #20400 kN
            elif self.bearing_type == 'SRB':
                self.mass = 2960 #21200 kN
            elif self.bearing_type == 'TRB':
                self.mass = 1061.43 #7550 KN

        #print self.mass
        self.mass += self.mass*(8000.0/2700.0) # add housing weight
        print ("MB Mass, kg: {0}").format(self.mass)

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
        
    #     d_cm_d_rotorDiameter = np.array([-0.035, 0.0, 0.025])
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
    #                        [0, d_cm_d_rotorDiameter[0]], \
    #                        [0, d_cm_d_rotorDiameter[1]], \
    #                        [0, d_cm_d_rotorDiameter[2]], \
    #                        [d_I_d_lssDiameter[0], 0], \
    #                        [d_I_d_lssDiameter[1], 0], \
    #                        [d_I_d_lssDiameter[2], 0]])

    # def provideJ(self):

    #     input_keys = ['lssDiameter', 'rotorDiameter']
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
        
#     #     d_cm_d_rotorDiameter = np.array([-0.01, 0.0, 0.025])
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
#     #                        [0, d_cm_d_rotorDiameter[0]], \
#     #                        [0, d_cm_d_rotorDiameter[1]], \
#     #                        [0, d_cm_d_rotorDiameter[2]], \
#     #                        [d_I_d_lssDiameter[0], 0], \
#     #                        [d_I_d_lssDiameter[1], 0], \
#     #                        [d_I_d_lssDiameter[2], 0]])

#     # def provideJ(self):

#     #     input_keys = ['lssDiameter', 'rotorDiameter']
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
    gearRatio = Float(iotype='in', desc='overall gearbox speedup ratio')
    #check on how to define array input in openmdao
    Np = Array(np.array([0.0,0.0,0.0,]), iotype='in', desc='number of planets in each stage')
    rotorSpeed = Float(iotype='in', desc='rotor rpm at rated power')
    rotorDiameter = Float(iotype='in', desc='rotor diameter')
    rotorTorque = Float(iotype='in', units='N*m', desc='rotor torque at rated power')

    #parameters
    #name = Str(iotype='in', desc='gearbox name')
    gearConfiguration = Str(iotype='in', desc='string that represents the configuration of the gearbox (stage number and types)')
    #eff = Float(iotype='in', desc='drivetrain efficiency')
    ratioType = Str(iotype='in', desc='optimal or empirical stage ratios')
    shType = Str(iotype='in', desc = 'normal or short shaft length')

    # outputs
    stageMasses = Array(np.array([0.0, 0.0, 0.0, 0.0]), iotype='out', units='kg', desc='individual gearbox stage masses')
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
        gearRatio : float
          Overall gearbox speedup ratio.
        gearConfiguration : str
          String describing configuration of each gear stage.  Use 'e' for epicyclic and 'p' for parallel, for example 'eep' would be epicyclic-epicyclic-parallel.  'eep_3' and 'eep_2 are also options that fix the final stage ratio at 3 or 2 respectively.
        Np : array
          Array describing the number of planets in each stage.  For example if gearConfig is 'eep' Np could be [3 3 1].
        rotorSpeed : float
          Rotational speed of the LSS at rated power [rpm].
        eff : float
          Mechanical efficiency of the gearbox.
        ratioType : str
          Describes how individual stage ratios will be calculated.  Can be 'empirical' which uses the Sunderland model, or 'optimal' which finds the stage ratios that minimize overall mass.
        shType : str
          Describes the shaft type and applies a corresponding application factor.  Can be 'normal' or 'short'.
        rotorTorque : float
          rotor torque.
        '''
        super(Gearbox_drive,self).__init__()

    def execute(self):

        self.stageRatio=np.zeros([3,1])

        self.stageTorque = np.zeros([len(self.stageRatio),1]) #filled in when ebxWeightEst is called
        self.stageMass = np.zeros([len(self.stageRatio),1]) #filled in when ebxWeightEst is called
        self.stageType=self.stageTypeCalc(self.gearConfiguration)
        #print self.gearRatio
        #print self.Np
        #print self.ratioType
        #print self.gearConfiguration
        self.stageRatio=self.stageRatioCalc(self.gearRatio,self.Np,self.ratioType,self.gearConfiguration)
        #print self.stageRatio

        m=self.gbxWeightEst(self.gearConfiguration,self.gearRatio,self.Np,self.ratioType,self.shType,self.rotorTorque)
        self.mass = float(m)
        self.stageMasses=self.stageMass
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

        '''def rotorTorque():
            tq = self.gbxPower*1000 / self.eff / (self.rotorSpeed * (pi / 30.0))
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
            Kgamma=1.25
        elif indNp == 5:
            Kgamma=1.35

        if indStageType == 1:
            indStageMass=1.0+indStageRatio+indStageRatio**2+(1.0/indStageRatio)

        elif indStageType == 2:
            sunRatio=0.5*indStageRatio - 1.0
            indStageMass=Kgamma*((1/indNp)+(1/(indNp*sunRatio))+sunRatio+sunRatio**2+Kr*((indStageRatio-1)**2)/indNp+Kr*((indStageRatio-1)**2)/(indNp*sunRatio))

        return indStageMass
        
    def gbxWeightEst(self, config,overallRatio,Np,ratioType,shType,torque):


        '''
        Computes the gearbox weight based on a surface durability criteria.
        '''

        ## Define Application Factors ##
        #Application factor for weight estimate
        Ka=0.6
        Kshaft=0.0
        Kfact=0.0

        #K factor for pitting analysis
        if self.rotorTorque < 200000.0:
            Kfact = 850.0
        elif self.rotorTorque < 700000.0:
            Kfact = 950.0
        else:
            Kfact = 1100.0

        #Unit conversion from Nm to inlb and vice-versa
        Kunit=8.029

        # Shaft length factor
        if self.shType == 'normal':
            Kshaft = 1.0
        elif self.shType == 'short':
            Kshaft = 1.25

        #Individual stage torques
        torqueTemp=self.rotorTorque
        for s in range(len(self.stageRatio)):
            #print torqueTemp
            #print self.stageRatio[s]
            self.stageTorque[s]=torqueTemp/self.stageRatio[s]
            torqueTemp=self.stageTorque[s]
            self.stageMass[s]=Kunit*Ka/Kfact*self.stageTorque[s]*self.stageMassCalc(self.stageRatio[s],self.Np[s],self.stageType[s])
        
        gbxWeight=(sum(self.stageMass))*Kshaft
        
        return gbxWeight

    def stageRatioCalc(self, overallRatio,Np,ratioType,config):
        '''
        Calculates individual stage ratios using either empirical relationships from the Sunderland model or a SciPy constrained optimization routine.
        '''

        K_r=0
                    
        #Assumes we can model everything w/Sunderland model to estimate speed ratio
        if ratioType == 'empirical':
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
        
        elif ratioType == 'optimal':
            x=np.zeros([3,1])

            if config == 'eep':
                x0=[overallRatio**(1.0/3.0),overallRatio**(1.0/3.0),overallRatio**(1.0/3.0)]
                B_1=Np[0]
                B_2=Np[1]
                K_r1=0
                K_r2=0 #2nd stage structure weight coefficient

                def volume(x):
                    return (1.0/(x[0]))*((1.0/B_1)+(1.0/(B_1*((x[0]/2.0)-1.0)))+(x[0]/2.0-1.0)+(x[0]/2.0-1)**2+K_r1*((x[0]-1.0)**2)/B_1 + K_r1*((x[0]-1.0)**2)/(B_1*(x[0]/2.0-1.0))) + (1.0/(x[0]*x[1]))*((1.0/B_2)+(1/(B_2*((x[1]/2.0)-1.0)))+(x[1]/2.0-1.0)+(x[1]/2.0-1.0)**2.0+K_r2*((x[1]-1.0)**2.0)/B_2 + K_r2*((x[1]-1.0)**2.0)/(B_2*(x[1]/2.0-1.0))) + (1.0/(x[0]*x[1]*x[2]))*(1.0+(1.0/x[2])+x[2] + x[2]**2)                              
                
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
        
            else:
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
    towerTopDiameter = Float(iotype ='in', desc='diameter of the top tower section at the yaw gear')
    shaftLength = Float(iotype = 'in', desc='LSS length')
    rotorDiameter = Float(iotype = 'in', desc='rotor diameter')

    #parameters
    #check openmdao syntax for boolean
    uptowerTransformer = Bool(iotype = 'in', desc = 'Boolean stating if transformer is uptower')

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
        towerTopDiameter : float
          Diameter of the top tower section at the nacelle flange [m].
        shaftLength : float
          Length of the LSS [m]
        rotorDiameter : float
          The wind turbine rotor diameter [m].
        uptowerTransformer : int
          Determines if the transformer is uptower ('1') or downtower ('0').
        '''

        #Treat front cast iron main beam in similar manner to hub
        #Use 1/4 volume of annulus given by:
        #length is LSS length plus tower radius
        #diameter is 1.25x tower diameter
        #guess at initial thickness same as hub
        #towerTopDiam, shaftLength, rotorDiameter, uptowerTransformer=0

        super(Bedplate_drive,self).__init__()

    def execute(self):
        castThickness=self.rotorDiameter/620
        castVolume=(self.shaftLength+self.towerTopDiameter/2)*pi*(self.towerTopDiameter*1.25)*0.25*castThickness
        castDensity=7.1*10.0**3 #kg/m^3
        castMass=castDensity*castVolume
        self.length=0.0
        self.width=1.2
        self.depth=0.66
        
        #These numbers based off V80, need to update
        steelVolume=0.0
        if self.uptowerTransformer == True:
            self.length=1.5*self.shaftLength
            steelVolume=self.length*self.width*self.depth/4.0
        elif self.uptowerTransformer == False:
            self.length=self.shaftLength
            steelVolume=self.length*self.width*self.depth/4.0
        steelDensity=7900 #kg/m**3
        steelMass=steelDensity*steelVolume

        self.mass = steelMass + castMass

        # calculate mass properties
        cm = np.array([0.0,0.0,0.0])
        cm[0] = cm[1] = 0.0
        cm[2] = 0.0122 * self.rotorDiameter                             # half distance from shaft to yaw axis
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
    rotorDiameter = Float(iotype='in', units='m', desc='rotor diameter')
    rotorThrust = Float(iotype='in', units='N', desc='maximum rotor thrust')
    towerTopDiameter = Float(iotype='in', units='m', desc='tower top diameter')
    aboveYawMass = Float(iotype='in', units='kg', desc='above yaw mass')

    #parameters
    numYawMotors = Float(iotype='in', desc='number of yaw motors')

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
        rotorDiameter : float
          Rotor Diameter [m].
        numYawMotors : int
          Number of yaw motors.
        '''
        super(YawSystem_drive, self).__init__()

    def execute(self):

        if self.numYawMotors == 0 :
          if self.rotorDiameter < 90.0 :
            self.numYawMotors = 4.0
          elif self.rotorDiameter < 120.0 :
            self.numYawMotors = 6.0
          else:
            self.numYawMotors = 8.0


        
        #assume friction plate surface width is 1/10 the diameter
        #assume friction plate thickness scales with rotor diameter
        frictionPlateVol=pi*self.towerTopDiameter*(self.towerTopDiameter*0.10)*(self.rotorDiameter/1000.0)
        steelDensity=8000.0
        frictionPlateMass=frictionPlateVol*steelDensity
        
        #Assume same yaw motors as Vestas V80 for now: Bonfiglioli 709T2M
        yawMotorMass=190.0
        
        totalYawMass=frictionPlateMass + (self.numYawMotors*yawMotorMass)
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
