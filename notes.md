# PID controller system architecture
The system consists of two components:
- A simulator which was developed by Udacity in Unity and simulates/displays our car, driving on a track around,
- And a PID controller which directs the car by controlling its steering wheel angle and throttle (gas pedal) in every time slice.
  There is a websocket (http-based data communication protocol) based communication between them. 
The development of the latter component was my project task in the Self-Driving Car Engineer Nanodegree Program. So, in the following part of this document I will describe that.

---

## PID controller mechanism and its design

The PID controller listens on the predefined TCP port 4567, and the simulator connects to it at the beginning. 
Ater the connection is established, the simulator sends JSON encoded messages with the current state of the car, and expects reply with the new control values in a similar JSON format message reply. This message exchange repeats frequently, controlled by the simulator logic.      
The message from the simulator contains the following data fields:
- CTE - The cross-track error is an error value which is proportional to the (signed, direction-dependent) distance of the car from the center of the road.
- speed - Current speed of the car
- steering_angle - Current steering angle of the car, in degrees
  The CTE value is the most important to calculate the new steering angle, as it's the current error value. 
  the algorithm uses the _steering_angle_ value only in the _Cost_ function of PIDTRAINING when it's configured to do so, and the _speed_ parameter is used 
    - during training, to calculate the average speed of the car and log it,
    - in the _Cost_ function of PIDTRAINING when it is used as extra weight value, 
    - and in the calculation in D and I error, in the role of dT. (dT is proportional to the reciprocal of the car's speed, so 1/speed is used there).
The reply message contains:
- steering_angle - Steering wheel angle to be used, in a [-1,1] interval. 
- throttle - Gas pedal throttle value to be used in the simulator.

The controller which calculates the values to put into the reply package is a PID controller (Proportional-integral-derivative controller). It's a control mechanism used in applications requiring continuous modulated control. It minimizes the deviation from a reference value by employing feedbacks continously.
As the name implies, to calculate this feedback the controller combines proportional control (P) with integral (I) and derivative (D) adjustments.   
This PID controller itself is implemented by the PID class. The controller needs 3 hyperparameters which are the coefficients used for the error calculation of the P, I and D components. 
The feedback value of the controller will be the sum of these 3 component errors. (Perror+Ierror+Derror)
- The P component's error will be proportional to the CTE itself,  
- D's will be proportional to the change of the CTE value,
- And I will be proportional to the sum of the errors (CTE values).

P is just trying to minimize the error by giving back a corrective value in the opposite direction of the deviation. As systems always have some inertia or other delay to react to the new controls, this P controller alone often overshoots the set value. That's why we need the D component which decreases oscillation by decreasing the correction in case of bigger changes in the CTE value. 
The final I component will enhance this further by increasing the corrective value when the system does not move, or moves too slowly toward to the correct value. (the CTE does not disappear in time, which also can happen in our car control case when there is a bias in the steering angle / real direction)    

## Hyperparameter tuning, optimization.

I am using 2 PI controllers: One for the steering angle and one for speed. Initially the latter was a P controller (I and D were 0 as those are not required at the throttle/speed control in my opinion) but later I found it much better to change it to a binary "logic": If the speed is below _optimal_speed_ the throttle=predefined_max_throttle, otherwise throttle=0. I achieved this by using this PID controller setup:
        pid_throttle.Init(999999, 0, 0);      
Here, the P coefficient is a big number. Moreover I limit the PID's returned _error_ between 0 lower and predefined_max_throttle upper bound.

To find the best coefficients for the steering angle PID controller, I have used many methods:
- Manual: Finding a good enough P value manually is not too hard, as the amplitude of the oscillation and the sustain/increase of it in time is visible in the simulator. After that the D coefficient can be guessed by trial and error and by checking the the aplitude of the oscillations when the car tries to go back inside toward the center of the road. If there is no overshoot, or it's very small, then the D value is good. I was using 0 for the I coefficient during the manual tuning.
- Auto: For this I am using the PIDTRAINER class. This tuning is active when _USE_TRAINING_ is #define -d at the beginning of pid.h. For auto tuning, I need to use the command line of the main executable, and give 6 floating point values to the main procedure: Pc, Ic, Dc, PDelta, IDelta, DDelta. These will be used in the twiddle algorithm which executes many simulations on a predefined part of the race track and tries to modify the 3 parameters with their corresponding delta values and find the best combination where the average of all PID errors is minimal. (this cost value to be minimized is the return value of the _loss function_ PID::GetCostValue() ) This method is useful to finetune the PID controller coefficients.              
- Auto, with Speed as error (used when USE_SPEED_WEIGHT is #defined): I've tried to add speed_errors to the PID::GetCostValue() summarized error value used in the twiddle algorithm ( spd_error = 2000*exp( -speed/50.0 ); ). (the error magnitude and the speed value are inversely proportional). Here I wanted the car to go faster, and wished that the oscillations would also be smaller. However It didn't improve too much, so I have rarely used this weighting. 
- Auto, with Angle as error (used when USE_ANGLE_WEIGHT is #defined): I've tried to add steering_angle to the PID::GetCostValue() summarized error value used in the twiddle algorithm ( 1000 * ( 1 - exp(-abs(angle) / 25.0) ); ). I wanted the control to direct a car more smoothly, so I added this weight based on the absolute value of the steering angle. I also used this rarely, because it did not result any visible improvement.

All these methods were used to find the final hyperparameters.  

The logging of the PIDTRAINER can be switched on by #define-ing USE_LOGGING at the beginning of PID.h. When it's on, the application will create a log.txt log file in the working directory with many useful information about the steps of the twiddle algorithm, the scores after each run, and the best PID controller parameters if they are found. 
 
## Other: Problems, issues, possible future enhancements/ideas

* The automatic twiddle algorithm could not be used for a long time because of the simulator, as it hangs (does not react to any user input and does not connect) after about half a day. I was using the _magic_ '42["reset",{}]' message to restart the simulator everytime, maybe it was not tested ? 
* json.hpp could not be used with the newest STL on windows, as it's a very old version (2.1.1). I had to copy the json.hpp from the CarND-Path-Planning project, it's newer and it compiles correctly (it's version is 3.0.0). 
* Sometime the websocket connection handshaking fails, and the connection forcibly closed by the server (PID controller app). The cause of this is that the maximum message length ( payload ) is by default only 16Kbytes in the uwebsockets implementation. If I start the simulator first, then the PID controller a little bit later, it often led to failed connection attempts. I have fixed this for the newest version used on my local windows machine ( when UWS_VCPKG is defined in the beginning of main.cpp ) but the Udacity version still contain this error.      
* The simulator in the Udacity workspace, which is used from Web Browser bahaves differently, probably because it's too slow. Even in the Fastest configuration. In a Local environment, the simulation with an average speed of 50 MPh could easily be done, but in the noVNC Simulator it failed, the car left the track, so currently I limit it's speed to 30 MPh before submitting this project.    

 
