# Lab 3: Wall Following

## I. Learning Goals

- PID Controllers
- Driving the car autonomously via Wall Following

## II. Review of PID in the time domain

A PID controller is a way to maintain certain parameters of a system around a specified set point. PID controllers are used in a variety of applications requiring closed-loop control, such as in the VESC speed controller on your car.

The general equation for a PID controller in the time domain, as discussed in lecture, is as follows:

$$ u(t)=K_{p}e(t)+K_{i}\int_{0}^{t}e(t^{\prime})dt^{\prime}+K_{d}\frac{d}{dt}(e(t)) $$

Here, $K_p$, $K_i$, and $K_d$ are constants that determine how much weight each of the three components (proportional, integral, derivative) contribute to the control output $u(t)$. $u(t)$ in our case is the steering angle we want the car to drive at. The error term $e(t)$ is the difference between the set point and the parameter we want to maintain around that set point.

## III. Wall Following

In the context of our car, the desired distance to the wall should be our set point for our controller, which means our error is the difference between the desired and actual distance to the wall. This raises an important question: how do we measure the distance to the wall, and at what point in time? One option would simply be to consider the distance to the right wall at the current time $t$ (let's call it $D_t$). Let's consider a generic orientation of the car with respect to the right wall and suppose the angle between the car's x-axis and the axis in the direction along the wall is denoted by $\alpha$. We will obtain two laser scans (distances) to the wall:
one 90 degrees to the right of the car's x-axis (beam b in the figure), and one (beam a) at an angle $\theta$ ( $0<\theta\leq70$ degrees) to the first beam. Suppose these two laser scans return distances a and b, respectively.

![fig1](img/wall_following_lab_figure_1.png)

*Figure 1: Distance and orientation of the car relative to the wall*

Using the two distances $a$ and $b$ from the laser scan, the angle $\theta$ between the laser scans, and some trigonometry, we can express $\alpha$ as

$$ \alpha=\mbox{tan}^{-1}\left(\frac{a\mbox{cos}(\theta)-b}{a\mbox{sin}(\theta)}\right) $$

We can then express $D_t$ as 

$$ D_t=b\mbox{cos}(\alpha) $$

to get the current distance between the car and the right wall. What's our error term $e(t)$, then? It's simply the difference between the desired distance and actual distance! For example, if our desired distance is 1 meter from the wall, then $e(t)$ becomes $1-D_t$.
	
However, we have a problem on our hands. Remember that this is a race: your car will be traveling at a high speed and therefore will have a non-instantaneous response to whatever speed and servo control you give to it. If we simply use the current distance to the wall, we might end up turning too late, and the car may crash. Therefore, we must look to the future and project the car ahead by a certain lookahead distance (let's call it $L$). Our new distance $D_{t+1}$ will then be

$$D_{t+1}=D_t+L\mbox{sin}(\alpha)$$

![fig1](img/wall_following_lab_figure_2.png)

*Figure 2: Finding the future distance from the car to the wall*

We're almost there. Our control algorithm gives us a steering angle for the VESC, but we would also like to slow the car down around corners for safety. We can compute the speed in a step-like fashion based on the steering angle, or equivalently the calculated error, so that as the angle exceeds progressively larger amounts, the speed is cut in discrete increments. For this lab, a good starting point for the speed control algorithm is:

- If the steering angle is between 0 degrees and 10 degrees, the car should drive at 1.5 meters per second.
- If the steering angle is between 10 degrees and 20 degrees, the speed should be 1.0 meters per second.
- Otherwise, the speed should be 0.5 meters per second.

So, in summary, here's what we need to do:

1. Obtain two laser scans (distances) a and b.
2. Use the distances a and b to calculate the angle $\alpha$ between the car's $x$-axis and the right wall.
3. Use $\alpha$ to find the current distance $D_t$ to the car, and then $\alpha$ and $D_t$ to find the estimated future distance $D_{t+1}$ to the wall.
4. Run $D_{t+1}$ through the PID algorithm described above to get a steering angle.
5. Use the steering angle you computed in the previous step to compute a safe driving speed.
6. Publish the steering angle and driving speed to the `/drive` topic in simulation.

## IV. Implementation

Implement wall following to make the car drive autonomously around the Levine Hall map. Follow the inner walls of Levine. Which means follow left- make sure the the car is going counter-clockwise in the loop. (The first race we run will be counter-clockwise). You can implement this node in either C++ or Python.

## V. Deliverables and Submission

**Deliverable 1**: After you're finished, update the entire skeleton package directory with your team's `wall_follow` package and directly commit and publish to GitHub. You will need to make sure the TA is added as a collaborator. Your committed code should start and run in simulation smoothly.The package should include a launch file that allows students to change variables from the command line for the following:
- P, I, and D tuning variables
- Vehicle Speed

**Simulator Demonstration**: Students will be required to demonstrate their team's implementation in class on the specified evaluation date. Team's will test their implementation in the Levine Hall (default) map. A successful demonstration includes:

- The vehicle autonomously drives a lap, following the left wall, around the track without collision.
- The vehicle can drive with imperceptible oscillations after turning the first corner.
- The vehicle can successfully make it past the trap in the bottom of the simulator map.

**On Vehicle Demonstration:** The presentation on vehicle will be held in person on a track set up in the classroom. The vehicle will be expected to complete the following tasks:
- The vehicle autonomously drives a lap, following the left wall, around the track without collision.
- The vehicle can drive with imperceptible oscillations after turning the first corner.
- The vehicle can successfully make it past traps located on the track.

## VI: Grading Rubric

- Launch File: **10** Points
- Implemented PID: **10** Points
- Simulator Demonstration: **40** Points
- Vehicle Demonstration: **40** Points

## VII: Extras
Some things to note during implementation:

-  Continual terminal output delays node processing and may impact the correct operation of the vehicle.
Provide terminal output for the successful launching of the node and no more.

### Clamping
Clamping is used to control the calculated control output, *u(t)*, which is to be considered the steering_angle.
While testing, it is possible this number will quickly get out of hand due to integral windup. Using a clamp
will ensure *u(t)* is always within an acceptable range.
The most the steering_angle can turn is by 20 degrees, in both directions. A negative steering_angle
will turn your vehicle right, and a positive angle to the left. The steering_angle is stored under the
AckermannDrive in radians. Be sure to keep this in mind while progressing through the lab.

### The Trap
On the South facing side of the simulator map lays a rectangular irregularity, illustrated below. This portion of the map is a trap. It is used to test implementations and catch the vehicle within its
confines. Due to the tight corners, it can be nearly impossible for the vehicle to maneuver out once it has
entered.

<img src="img/trap.PNG" width=50% height=50%>
