# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---
# P Component
The P component in PID is for Proportional control. 
Using propotional band, the controller output is proportional to the error. In the PID project, the error is the CTE, which refers to the offset of the vehicle from the center of the road.

Controller output = (CTE) * Kp (Here Kp is the proportional constant)

---
# I Component
The I component in PID is for Integral control. 
Using integral band, the controller output is proportional to the error accumulated over time. It is used to remmove offset.

Controller output = (Accumulated Error) * Ki (Here Ki is the integral constant)

---
# D Component
The D component in PID is for Derivative control. 
Using derivative band, the controller output is proportional to the rate of change of error. It is used to compensate for a changing measurement. Derivative controller takes action to inhibit more rapid changes of the measurement than compared to proportional action. It is often used to avoid overshoot.

Controller output = (Change in Error) * Kd (Here Kd is the Derivate constant)

---

# Tuning PID
The effective PID for the project was determined using a combination of manual tuning and twiddle. Manually initial PID parameters were set. Then twiddle was used to fine tune them. Every 100 steps the value of individual parameters were modified and depending on change in error, the parameter values were set. This was done till the parameter change values were within a threshold of 0.2.

---
# Effect of Components in Implementation
When the only proportional control was used, the car always aimed at reaching the state of zero error, and this lead it to overhsoot each to the other extreme. This process continued, leading to pendulum motion.

Addition of Differential control lead to a smoother movement. The Vehicle tended to stay at center and there a reduction in the ovehooting of the vehicle.

Integral Control was added to remove offset of the vehicle over time. After adding this, the vehice tended to stay in the center of the path.  
