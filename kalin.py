import numpy as np
import matplotlib.pyplot as plt

import python_vehicle_simulator.lib as pvsl
import python_vehicle_simulator.vehicles as pvsv

def main():
    vehicle = pvsv.remus100("depthHeadingAutopilot", r_rpm=1525)
    delta_t = 0.01
    simdur1 = 10
    ts1 = np.arange(0, simdur1+delta_t, delta_t)

    DOF = 6                     # degrees of freedom
    t = 0                       # initial simulation time
    # Initial state vectors
    eta = np.array([0, 0, 0, 0, 0, 0], float)    # position/attitude, user editable
    nu = vehicle.nu                              # velocity, defined by vehicle class
    u_actual = vehicle.u_actual                  # actual inputs, defined by vehicle class
    # Initialization of table used to store the simulation data
    simData = np.empty( [0, 2*DOF + 2 * vehicle.dimU], float)
    # Simulator for-loop
    for t in ts1:        
        # Vehicle specific control systems
        u_control = vehicle.depthHeadingAutopilot(eta,nu,delta_t)             
        
        # Store simulation data in simData
        signals = np.append( np.append( np.append(eta,nu),u_control), u_actual )
        simData = np.vstack( [simData, signals] ) 

        # Propagate vehicle and attitude dynamics
        [nu, u_actual]  = vehicle.dynamics(eta,nu,u_actual,u_control,delta_t)
        eta = pvsl.attitudeEuler(eta,nu,delta_t)

    # Change desired depth to 30 meters and run simulation for thirty more seconds
    vehicle.ref_z = 10
    simdur2 = 240
    ts2 = np.arange(delta_t, simdur2+delta_t, delta_t) + ts1[-1]
    # Simulator for-loop
    for t in ts2:        
        # Vehicle specific control systems
        u_control = vehicle.depthHeadingAutopilot(eta,nu,delta_t)             
        
        # Store simulation data in simData
        signals = np.append( np.append( np.append(eta,nu),u_control), u_actual )
        simData = np.vstack( [simData, signals] ) 

        # Propagate vehicle and attitude dynamics
        [nu, u_actual]  = vehicle.dynamics(eta,nu,u_actual,u_control,delta_t)
        eta = pvsl.attitudeEuler(eta,nu,delta_t)

    # Change desired heading to 180 degrees and run simulation for thirty more seconds
    vehicle.ref_psi = 10
    simdur3 = 60
    ts3 = np.arange(delta_t, simdur3+delta_t, delta_t) + ts2[-1]
    # Simulator for-loop
    for _ in ts3:        
        # Vehicle specific control systems
        u_control = vehicle.depthHeadingAutopilot(eta,nu,delta_t)             
        
        # Store simulation data in simData
        signals = np.append( np.append( np.append(eta,nu),u_control), u_actual )
        simData = np.vstack( [simData, signals] ) 

        # Propagate vehicle and attitude dynamics
        [nu, u_actual]  = vehicle.dynamics(eta,nu,u_actual,u_control,delta_t)
        eta = pvsl.attitudeEuler(eta,nu,delta_t)

    # Store simulation time vector
    simTime = np.vstack((ts1[:, None], ts2[:,None], ts3[:,None]))

    pvsl.plotVehicleStates(simTime, simData, 1)                    
    pvsl.plotControls(simTime, simData, vehicle, 2)
    numDataPoints = 100                  # number of 3D data points
    FPS = 10                            # frames per second (animated GIF)
    filename = '3D_animation.gif'       # data file for animated GIF
    pvsl.plot3D(simData, numDataPoints, FPS, filename, 3)
    plt.show()
    return

if __name__ == "__main__":
    main()