# Model Predictive Control Project Reflection - Braulio Chavez

## The Model
State for vehicle's model:

* x: position of the vehicle in the forward direction
* y: position of the vehicle in the lateral direction
* psi: yaw angle or orientation of the vehicle
* v: speed of the vehicle

Actuators:

* delta: steering angle in radians
* a: acceleration in m/s^2

Update equations to calculate state in next time step using the current
state.

```
px = px + v * cos(psi) * dt;
psi = psi - v * steer_value / Lf * dt;
v = v + throttle_value * dt;
```

* Lf: Length from front to center of gravity of vehicle.
* dt: timestep difference.

## Timestamp Length and Elapsed Duration (N & dt)
I started with dt=0.1 (which is 100 milliseconds) proved to be too long
between timesteps the vehicle had trouble in curves since the
calculation of what to do came too late when the car was already on a
different position, causing the car to oscillate and some times get off
the track. This problem could have been solved if I configured the
vehicle to go at a slower speed on curves to compensate for slow
computation.

Then I tried with dt=0.05 (50 milliseconds) this turned out to be way
better for the vehicles performanc, having much smoother curves at a
good speed.

I started with a timestamp length of N=25. This number was too high
since I noticed that the computations were arriving late to the vehicle
again due to high speed. I decided to reduce N=20 arriving to a better
performance and maintaining a good speed ~40mph.

## Polynomial Fitting and MPC Preprocessing
The waypoints are transformed to the car coordinate system (described
above in the vehicle's state variables). Initially the values for x, y
and psi can be set to zero because the coordinate system is relative to
the car.

```
double dx = ptsx[i] - px;
double dy = ptsy[i] - py;
waypoints_x.push_back(dx * cos(-psi) - dy * sin(-psi));
waypoints_y.push_back(dx * sin(-psi) + dy * cos(-psi));
```

Before processing the waypoints I take latency into consideration. How
latency is handled is described in the section below.

## Model Predictive Control with Latency
The project introduces a latency of 100 milliseconds in the `main.cpp`
file. This latency will affect the computations given that when the
model calculates the new steering angle this computation will be already
in the past, subsequent runs of the model will try to compensate for
that error causing the car to oscillate, specially at high speeds.

There are two methods to compensate for latency that I though of.
1. Start your initial state computation considering the latency.
   Predicting the future state of the model.
2. Average your new model result with previous results.

I chose to do number `1` given that it is easier to implement and
produces good behavior. This compensation is given initally before
computing the polynomial fitting and waypoint calculation. Here's the
code that takes it into account.

```
// Account for Latency
px = px + v * cos(psi) * latency;
psi = psi - v * steer_value / Lf * latency;
v = v + throttle_value * latency;

vector<double> waypoints_x;
vector<double> waypoints_y;
```

## Video
You can find my model running in the simulator on this video:
[MPC Video](https://youtu.be/TEph06sNuPk)
