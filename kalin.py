import numpy as np
import matplotlib.pyplot as plt

import python_vehicle_simulator.lib as pvsl
import python_vehicle_simulator.vehicles as pvsv

def main():
    vehicle = pvsv.remus100("depthHeadingAutopilot", r_rpm=1525)
    delta_t = 0.01
    total_sim_duration = 10
    num_time_steps = total_sim_duration / delta_t

    DOF = 6                     # degrees of freedom
    t = 0                       # initial simulation time
    # Initial state vectors
    eta = np.array([0, 0, 0, 0, 0, 0], float)    # position/attitude, user editable
    nu = vehicle.nu                              # velocity, defined by vehicle class
    u_actual = vehicle.u_actual                  # actual inputs, defined by vehicle class
    # Initialization of table used to store the simulation data
    simData = np.empty( [0, 2*DOF + 2 * vehicle.dimU], float)
    # Simulator for-loop
    for i in range(0,num_time_steps+1):
        
        t = i * delta_t      # simulation time
        
        # Vehicle specific control systems
        u_control = vehicle.depthHeadingAutopilot(eta,nu,delta_t)             
        
        # Store simulation data in simData
        signals = np.append( np.append( np.append(eta,nu),u_control), u_actual )
        simData = np.vstack( [simData, signals] ) 

        # Propagate vehicle and attitude dynamics
        [nu, u_actual]  = vehicle.dynamics(eta,nu,u_actual,u_control,delta_t)
        eta = attitudeEuler(eta,nu,delta_t)

    # Store simulation time vector
    simTime = np.arange(start=0, stop=t+delta_t, step=delta_t)[:, None]

    plotVehicleStates(simTime, simData, 1)                    
    plotControls(simTime, simData, vehicle, 2)
    plot3D(simData, numDataPoints, FPS, filename, 3)
    return

if __name__ == "__main__":
    main()