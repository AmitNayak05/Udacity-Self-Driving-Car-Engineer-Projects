# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Problem Statement
The aim of the project was to drive the car in the simulator using Model Predictive Control. Using this method,
we predict the future course of movement and determine the necessary control actions in order to have smooth movement of the vehicle. Also
in order to simulate real time experience, a 100 ms latency is introduced between actuation commands.

## Model
The vehicle model for the project is a bicycle model. We also use a kinematic model as it ignores tireforces, gravity and mass and are simplifications of dynamic model, and also as at low speeds both are similar. In order to keep track of the state 
of the vehicle, we have a state variable that is composed of 6 components: x, y, heading, speed, cross track error and heading
error.
The actuator input used to control the vehicle is represented by  [δ,a]. δ represents steering angle and a is used for acceleration.
We try to predict the future state based on previous state and current actuator inputs. The update equation are:

	
		 x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
		 y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
		 psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
		 v_[t+1] = v[t] + a[t] * dt
		 cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
		 epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt


## Timestep Length and Elapsed Duration (N & dt)
N is the timestep length and dt is the time elapse between actuations. T is defined as a product og N and st
and determines the duration over which future predictions are made. For the project, I have taken N=25 and dt as 
0.05, and hence am looking ahead 1.25 secs. Different values were tried out and this combination was found to be the most suitable.

## Polynomial Fitting and preprocessing
A set of points is received which contains a set of 6 waypoint coordinates. These coordinates are in global map coordinates.
For preprocessing, we convert these coordinates to vehicle frame of reference. This is done by taking
into consideration the translation and rotation of the vehicle. 


		Trannsformed_x_value = Waypoint_x_value - car_x
		Trannsformed_y_value = Waypoint_y_value - car_y

		converted_x=(Trannsformed_x_value*cos(-psi)) - (Trannsformed_y_value*sin(-psi));
		converted_y=(Trannsformed_y_value*cos(-psi)) + (Trannsformed_x_value*sin(-psi));


The point given by (converted_x,converted_y) gives the point in car coordinate frame.

Next we try to fit a 3rd order polynomial to these set of points. This is done by:


	auto coeffs = polyfit(xvals, yvals, 3);


This gives us the coefficients for the 3rd order polynomial. Then I use these points to find next 16 ponts,
in order to display the path. Code for it is:

	for(double x = 0; x <= 80; x += 5.0){
				auto v = polyeval(coeffs, x);
				double y_temp=v;
				next_x_vals.push_back(x);
				next_y_vals.push_back(y_temp);
		}

 
 For MPC preprocessing, we find the current cross track error and orientation error. This is done by:

		double cte=polyeval(coeffs, 0.0);
		double epsi = -atan(coeffs[1])

 
 As the vehicle is always the center around which the points are translated and rotated, the vehicle is always at (0,0)
The state is given by 

		state << 0, 0, 0, v, cte, epsi;


## Model Predictive Control with Latency

In order to make the controller work in presence of latency, an appropriate cost function was created,
which when minimized gave us the required steering and acceleration values. 


	fg[0] = 0;

	   //The part of the cost based on the reference state.
	    for (int i = 0; i < N; i++) {
	      fg[0] += 0.08*CppAD::pow(vars[cte_start + i] - ref_cte, 2);
	      fg[0] += CppAD::pow(vars[epsi_start + i] - ref_epsi, 2);
	      fg[0] += CppAD::pow(vars[v_start + i] - ref_v, 2);
	    }

	    // Minimize the use of actuators.
	    for (int i = 0; i < N - 1; i++) {
	      fg[0] += 1900.0*CppAD::pow(vars[delta_start + i], 2);
	      fg[0] += 1.0*CppAD::pow(vars[a_start + i], 2);
	    }

	    // Minimize the value gap between sequential actuations.
	    for (int i = 0; i < N - 2; i++) {
	      fg[0] += CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
	      fg[0] += CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
	    }

For higher speed movement, the weight of CTE part for cst fuction was reduced to afator of 0.08 and the 
steering increased by a factor of 1900.

In order to take care of sharp turns, I find a circle fitted to three point (etreme point nd center of the waypoint polynomial)
The radius tof the circle is also used to reduce the speed to help keep the vehicle in track.


		if(fabs(radius)<120){
			throttle_value=-0.01;
			}
		if(fabs(radius)>=120 && fabs(radius)< 260) {
			throttle_value=0.15;
			}


The video of the project can be seen at:
https://www.youtube.com/watch?v=svKQbFq5ddY








