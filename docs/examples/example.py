# 1 ---------

from math import pi
from drivewpact.hub import HubWPACT
from drivewpact.drive import DriveWPACT

# 1 ---------
# 2 ---------

# simple test of hub model

# NREL 5 MW turbine

hub = HubWPACT()
hub.blade_mass = 17740.0 # kg
hub.rotor_diameter = 126.0 # m
hub.blade_number  = 3
hub.blade_root_diameter   = 3.542
AirDensity= 1.225 # kg/(m^3)
Solidity  = 0.0517
RatedWindSpeed = 11.05 # m/s
hub.rotor_bending_moment = (3.06 * pi / 8) * AirDensity * (RatedWindSpeed ** 2) * (Solidity * (hub.rotor_diameter ** 3)) / hub.blade_number

# 2 ----------
# 3 ----------

hub.run()

# 3 ----------
# 4 ----------

print "Estimate of Hub Component Sizes for the NREL 5 MW Reference Turbine"
print "Hub Components"
print '  Hub: {0:8.1f} kg'.format(hub.hub.mass)  # 31644.47
print '  Pitch system: {0:8.1f} kg'.format(hub.pitchSystem.mass) # 17003.98
print '  Nose cone: {0:8.1f} kg'.format(hub.spinner.mass) # 1810.50
print 'Hub system total: {0:8.1f} kg'.format(hub.hub_system_mass) # 50458.95
print '    cm {0:6.2f} {1:6.2f} {2:6.2f} [m, m, m]'.format(hub.hub_system_cm[0], hub.hub_system_cm[1], hub.hub_system_cm[2])
print '    I {0:6.1f} {1:6.1f} {2:6.1f} [kg*m^2, kg*m^2, kg*m^2]'.format(hub.hub_system_I[0], hub.hub_system_I[1], hub.hub_system_I[2])
print

# 4 ---------
# 5 ---------

# test of drivetrain model

# NREL 5 MW Rotor Variables

nace = DriveWPACT()
nace.rotor_diameter = 126.0 # m
nace.rotor_speed = 12.1 # rpm m/s
nace.machine_rating = 5000.0
nace.DrivetrainEfficiency = 0.95
nace.rotor_torque =  1.5 * (nace.machine_rating * 1000 / nace.DrivetrainEfficiency) / (nace.rotor_speed * (pi / 30)) # 6.35e6 #4365248.74 # Nm
nace.rotor_thrust = 2.5448e5
nace.rotor_mass = 142585.75 #kg
nace.rotor_speed = 12.1 #rpm

# NREL 5 MW Drivetrain variables
nace.drivetrain_design = 'geared' # geared 3-stage Gearbox with induction generator machine
nace.gear_ratio = 96.76 # 97:1 as listed in the 5 MW reference document
nace.gear_configuration = 'eep' # epicyclic-epicyclic-parallel
nace.bevel = 0 # no bevel stage
nace.crane = True # onboard crane present

# NREL 5 MW Tower Variables
nace.tower_top_diameter = 3.78 # m

# 5 ---------
# 6 ---------

nace.run()

# 6 ---------
# 7 ---------

print "Estimate of Nacelle Component Sizes for the NREL 5 MW Reference Turbine"
print 'Low speed shaft: {0:8.1f} kg'.format(nace.lowSpeedShaft.mass)
print 'Main bearings: {0:8.1f} kg'.format(nace.mainBearing.mass + nace.secondBearing.mass)
print 'Gearbox: {0:8.1f} kg'.format(nace.gearbox.mass)
print 'High speed shaft & brakes: {0:8.1f} kg'.format(nace.highSpeedSide.mass)
print 'Generator: {0:8.1f} kg'.format(nace.generator.mass)
print 'Variable speed electronics: {0:8.1f} kg'.format(nace.above_yaw_massAdder.vs_electronics_mass)
print 'Overall mainframe:{0:8.1f} kg'.format(nace.above_yaw_massAdder.mainframe_mass)
print '     Bedplate: {0:8.1f} kg'.format(nace.bedplate.mass)
print 'Electrical connections: {0:8.1f} kg'.format(nace.above_yaw_massAdder.electrical_mass)
print 'HVAC system: {0:8.1f} kg'.format(nace.above_yaw_massAdder.hvac_mass )
print 'Nacelle cover: {0:8.1f} kg'.format(nace.above_yaw_massAdder.cover_mass)
print 'Yaw system: {0:8.1f} kg'.format(nace.yawSystem.mass)
print 'Overall nacelle: {0:8.1f} kg'.format(nace.nacelle_mass, nace.nacelle_cm[0], nace.nacelle_cm[1], nace.nacelle_cm[2], nace.nacelle_I[0], nace.nacelle_I[1], nace.nacelle_I[2]  )
print '    cm {0:6.2f} {1:6.2f} {2:6.2f} [m, m, m]'.format(nace.nacelle_cm[0], nace.nacelle_cm[1], nace.nacelle_cm[2])
print '    I {0:6.1f} {1:6.1f} {2:6.1f} [kg*m^2, kg*m^2, kg*m^2]'.format(nace.nacelle_I[0], nace.nacelle_I[1], nace.nacelle_I[2])
# 7 ---------