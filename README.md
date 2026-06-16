# Lyapunov based Spacecraft Attitude control

![[Pasted image 20260616141322.png|669]]
*The rates and attitude error (relative to reference body) going to 0, and the value of the Lyapunov function approach 0*
## 1. Overview
This project implements Lyapunov based control methods for spacecraft attitude tracking and stabilisation. It controls the rates (angular velocity) and the attitude, represented using MRPs and brings them to both to 0 relative to another coordinate frame depending upon the application. The entire flow has been made in MATLAB.

There are 2 main simulations: 
1. `detumbling.m` starts a spacecraft from some attitude and having a certain tumbling rate, and brings both the attitude and the rates to 0 relative to the fixed inertial frame.
2. `attitude_tracking.m` starts a spacecraft from some attitude and a certain tubming rate, and bring the attitude and rates to 0 relative to the coordinate frame of the object that needs to be tracked (e.g., another satellite for docking, a star system, etc.)

## 2. Files contained
1. **Simulation files:** the files that run the simulation and perform the integration, and store the data. This repo contains `detumbling.m` and `attitude_tracking.m`.
2. **Helper functions:** We have functions `DCM2MRP.m` and `MRP2DCM.m` that convert Directional Cosine Matrices (DCMs) to Modified Rodrigues Parameters (MRPs). The conversion makes the intermediate math steps easier. 
   We also have functions like skew and lyap, which are single-line functions defined within the files. Skew takes a vector and returns the equivalent skew symmetric cross-product matrix. Lyap takes in the parameters and returns the scalar value of the lyapunov function 
3. **Plotter:** this file takes the values of the rates, attitudes and lyapunov function at each time step (stored from the integrator in the simulation) and plots them, allowing for better visualisation of what the control is actually doing.


## 3. The Maths used

The control law used for bringing the rates and attitude to 0 is done based on  Lyapunov control theory.

A [Lyapunov function](https://en.wikipedia.org/wiki/Lyapunov_function) is a function used to check the stability of dynamical systems, without having to solve complex intercoupled differential equations. It essentially represents the "energy" (not really, but kind of) or the distance from the equilibrium.

For a system with state vector $x$, a Lyapunov function $V(x)$ can be chosen, such that:
1. $V(x) > 0$ and ALWAYS returns a scalar value for any x,
2. $V(0) = 0$, and 
3. $\dot{V} (x) \leq 0$  
If these conditions are satisfied, then the system remains bounded and converges towards the equilibrium point. If the Lyapunov function is chosen well, then it continuously decrease with time until the system reaches the desired state.

### Application to the Spacecraft problem
I have used Lyapunov functions in this case to guarantee stable attitude. A candidate Lyapunov function is constructed using the attitude error and the angular velocity terms. The control is then designed such that the derivative of the Lyapunov function (also called the Lyapunov rate) stays negative semidefinite, i.e., $$ \dot{V} (\omega, \sigma) \leq 0 ; \space \forall \sigma, \omega \neq 0 $$
This ensures that the system continuously dissipates energy , and the error terms go to 0.

For the Detumbling problem, the goal is to get $\omega \rightarrow 0$ and $\sigma \rightarrow 0$. here both $\sigma$ and $\omega$ represent the attitude and the rate between the Body and the Inertial frame respectively\
For the attitude tracking, we consider R to be the frame whatever we want to track. Then $\omega_{B/R} \rightarrow 0$ and $\sigma_{B/R} \rightarrow 0$.

The Lyapunov function for both of these is: 
$$ V(\omega, \sigma) = \frac{1}{2}\omega^T [I] \omega + 2K ln (1 + 2 \sigma^T \sigma)$$
here, $\omega$ and $\sigma$ and both 3x1 column vector, $[I]$ is the inertia tensor (symmetric) and K is a linear gain (for the feedback attitude control term)

then taking the derivative of this, we get;
$$ 
\dot{V} (\omega, \sigma)  = \omega^T([I] \dot{\omega} + K\sigma)$$
for stable control, we want this to be semidefinite. Set the Lyapunov rate to be equal to $$  
\dot{V}  
=  
-\omega^T[P]\omega  
$$where $[P]$ is a symmetric positive-definite matrix.    

The rotational dynamics of the spacecraft are governed by Euler's equation    
$$  
[I]\dot{\omega}  
=  
-\tilde{\omega}[I]\omega + u + L  
$$
Here, $u$ is the control torque vector (3x1), $L$ is the external disturbance torque and $\tilde{\omega}$ is the skew symmetric matrix associated with $\omega$. In our case we assume that we do not have disturbances. Thus $L = 0$ 

The attitude kinematics are described using Modified Rodrigues Parameters:    $$  
\dot{\sigma}  
=  
B(\sigma)\omega  
$$where $B(\sigma)$ is the MRP kinematic matrix. 

Substituting the equations of motion into the Lyapunov derivative gives:
$$  
\dot{V}  
=  
\omega^T  
\left(  
-\tilde{\omega}[I]\omega  
+  
u  
+  
K\sigma  
\right)  
$$
Using the property $\omega^T \tilde{\omega} = 0$ (since $\tilde{\omega}$ represents cross product, and a vector crossed with itself is 0), the gyroscopic term vanishes, leaving $$  
\dot{V}  
=  
\omega^T(u + K\sigma)  
$$But, $\dot{V} = -\omega^T[P]\omega$. Since $[P]$ is positive definite, the Lyapunov function is non-increasing along all system trajectories. This guarantees stable convergence of the angular velocity and attitude error to their desired equilibrium values.  

Solving for $u$ with the above equations, we get: $u = -[P]\omega - K\sigma$ 
Mathematically, we also get the control:  $u = -[P]\omega - K\sigma +\tilde{\omega}[I]\omega$

Both of these will control the attitude and rates well, but the difference is in the internal closed loop dynamics. 

## 4. Running the Sim
The procedure is the same in both cases.
1. Set the initial attitude and rate in at the top of `attitude_tracking.m`/`detumbling.m`. Also set the inertia tensor.
2. Set the runtime using the `T` variable, and length of timestep  `dt`  (in seconds)
3. Run the simulations. The state $(\omega, \sigma$) is stored at every timestep in the integration loop in the variables `rates` and `attitude` 
   We also calculate the value of the Lyap function and store it in `V`.
4. Once the sim is complete, the variables will be stored in the workspace. This will be used in the `plotting.m` file to display the results. Run the file normally to see the behaviour of the spacecraft.

## 5. (Expected) results
The expected behaviour of the controller is:
1. Lyap function approaches a minimum value or 0
2. The angular velocity $\omega$ and attitude error $\delta \sigma$ approach 0 and stay there.
3. Control remains stable for arbitrary initial attitudes and rates.

An example of the expected output is the image attached at the top.

## 6. Future changes
The current implementation assumes: 
- Perfectly rigid spacecraft body, with nonvarying inertia tensor
- No actuator saturation
- No sensor noise
- No disturbance torque

### Potential changes/improvements
Adding CMG/Reaction wheel dynamics
Actuator constraints
Sensor noise and state estimation
