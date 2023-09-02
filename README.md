# wall-follow-f1tenth
This project focuses on implementing a wall following mechanism using PID controllers. This document provides a comprehensive guide about the task, the underlying principles, and the deliverables.


## Overview

In this project, I successfully developed an autonomous wall-following mechanism for a car using PID controllers. The goal was to ensure that the car maintained a consistent distance from a wall, simulating real-world autonomous driving situations.

## Key Components

### PID Controllers

Using the principles of Proportional-Integral-Derivative (PID) controllers, I implemented a control system that adjusted the car's steering based on its distance from the wall. The controller equation I used is:

u(t) = Kp * e(t) + Ki * ∫[0,t] e(t') dt' + Kd * d/dt (e(t))


Where:
- `u(t)` is the steering angle.
- `e(t)` represents the error, or the difference between the desired and actual distance to the wall.
- `Kp`, `Ki`, and `Kd` are weights that I tuned for the proportional, integral, and derivative components to achieve optimal performance.

### Wall Following Technique

I incorporated a laser scanning mechanism to gauge the car's distance and orientation relative to a wall. By computing the angle α, I could determine the car's x-axis direction concerning the wall, allowing for accurate distance calculations, Dt and Dt+1.

![image](https://github.com/Saibernard/wall-follow-f1tenth/assets/112599512/451accb0-9a59-4c37-81b3-b4ac6100a971)


![image](https://github.com/Saibernard/wall-follow-f1tenth/assets/112599512/bd033bc5-2ab6-47ed-bb01-6bf2f9b52b79)


### Dynamic Speed Control

One of the crucial features I integrated was the dynamic speed control based on the computed steering angle. This feature adjusts the car's speed in real-time based on its angle concerning the wall, ensuring that the car slows down during tight corners for safety.

![image](https://github.com/Saibernard/wall-follow-f1tenth/assets/112599512/e3efdb6c-1490-481e-a276-aa6e4d45b656)

![image](https://github.com/Saibernard/wall-follow-f1tenth/assets/112599512/7f21c06c-832b-49d4-ad46-d9c5b64ed858)



## Implementation Strategy

1. Laser Distance Measurement: I began by obtaining two laser scans to accurately measure the car's distance to the wall.
2. Orientation Calculation: Using the laser scan data, I computed the angle α which represented the car's orientation concerning the wall.
3. Distance Estimations: I then calculated both the current distance Dt and the projected distance Dt+1 from the wall, allowing for anticipatory steering adjustments.
4. Steering with PID: Using the PID controller mechanism, I processed the projected distance Dt+1 to compute the ideal steering angle.
5. Speed Recommendations: Based on the derived steering angle, I implemented a mechanism to suggest an appropriate driving speed, ensuring safety during various steering scenarios.
6. Publication to Simulation: Finally, the computed steering angle and speed were published to the `/drive` topic, enabling the car to maneuver autonomously in the simulation environment.

This summary offers a comprehensive view of my technical involvement and achievements in the Wall Following Lab of an autonomous car. It emphasizes the technical nuances and strategies employed to ensure successful autonomous navigation of the car.


## Video link:

This video shows the effective wall following strategy

https://youtu.be/nHq3BgRAQy8
