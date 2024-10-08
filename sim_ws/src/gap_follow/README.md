# Lab 4: Follow the Gap

## I. Learning Goals

- Reactive methods for obstacle avoidance

## II. Overview

In this lab, you will implement a reactive algorithm for obstacle avoidance. While the base starter code defines an implementation of the F1TENTH Follow the Gap Algorithm, you are allowed to submit in C++, and encouraged to try different reactive algorithms or a combination of several. In total, the python code for the algorithm is only about 120 lines.

## III. Review of F1TENTH Follow the Gap

The lecture slides on F1TENTH Follow the gap is the best visual resource for understanding every step of the algorithm. However, the steps are outlined over here:

1. Obtain laser scans and preprocess them.
2. Find the closest point in the LiDAR ranges array.
3. Draw a safety bubble around this closest point and set all points inside this bubble to 0. All other non-zero points are now considered “gaps” or “free space”.
4. Find the max length “gap”, in other words, the largest number of consecutive non-zero elements in your ranges array.
5. Find the best goal point in this gap. Naively, this could be the furthest point away in your gap, but you can probably go faster if you follow the “Better Idea” method as described in lecture.
6. Actuate the car to move towards this goal point by publishing an `AckermannDriveStamped` to the /drive topic.

### IV. Implementation

Implement a gap follow algorithm to make the car drive autonomously around the Levine Hall map. You can implement this node in either C++ or Python. In the simulator, the vehicle will be tested
in two maps provided in the gap_follow node: `levine_blocked.png`, which is empty, and `levine_obs.png`, which has obstacles that are relatively hard to navigate through for you to evaluate your code on.

```bash
1 gordon@f1sim:~/ws/gym-one/f1tenth_gym_ros/maps$ ls
2 levine_blocked.png levine_obs.png levine.png Spielberg_map.png
3 levine_blocked.yaml levine_obs.yaml levine.yaml Spielberg_map.yaml
```

To change the map in the simulation, add the included `.png` and `.yaml` map files to `f1tenth_gym_ros/maps` directory. Then, change `f1tenth_gym_ros/config/sim.yaml` to use your desired map.

```bash
1 gordon@f1sim:~/ws/gym-one/f1tenth_gym_ros/config$ emacs -nw sim.yaml # edit the sim.yaml file 
2 >>> SNIP <<<
3 # map parameters
4 map_path: '/sim_ws/src/f1tenth_gym_ros/maps/levine_obs'
5 map_img_ext: '.png'
6 >>> SNIP <<<
```
**After any changes to the sim.yaml file, the docker will need to be restarted, re-entered and rebuilt.**

### V. Deliverables and Demonstrations

**Deliverable 1**: After you're finished, update the entire skeleton package directory with your `gap_follow` package and directly commit and push to a repo shared with your TA. Your commited code should start and run in simulation smoothly.

**Shared Demonstration Requirements**:
The parameters to the follow the gap algorithm must be accepted as command line arguments. There may be default values provided by the launch file, but they must be able to be overridden on the command line. The purpose of this requirement is to allow students the opportunity to adjust their implementations for the track conditions. This Lab affords students flexibility in their method of gap following, which may require a distinct set of parameters per implementation. Include a help option to your launch file that will display the command line options and their meaning and then stops the system.

```# Breanna please include an example invocation, you do not need to implement this just a text sample such as:
# ros2 launch follow_gap.py --help
  bubble:=#   the size of the safety bubble in meters
  maxvel:=#   the maximum velocity of the vehicle in meters/sec
```

**Simulator Demonstration**:
A satisfactory demonstration includes the following:
- The vehicle autonomously drives a lap around the track without collision.
- The vehicle can successfully maneuver around obstacles on the track.
- A launch command that includes all parameters of the given obstacle avoidance algorithm

In presentation of the simulator, start your gap_follow node in an adjacent terminal to the simulator
preview. Have the node output a message to terminal indicating that the node has successfully been started.

*Note: Continual terminal output delays node processing and may impact the correct operation of the vehicle.
Provide terminal output for the successful launching of the node and no more.*

**Vehicle Demonstration**: The presentation on vehicle will be held on a track set up in the classroom. The vehicle will be expected
to complete the following tasks:
- The vehicle autonomously drives a lap around the track without collision.
- The vehicle can successfully make its way around obstacles on the track.
- A launch command that includes all parameters of the given obstacle avoidance algorithm

Depending on implementations, changes may need to be made to account for differences present on
vehicle that do not exist in simulation. An example might be a difference in the radius of the physical
vehicle, which is about 6 inches in length.



### VI. Grading Rubric

- Compilation: **10** Points
- Implemented Find-Max Gap: **40** Points
- Implemented Find best point: **30** Points
- Clears Levine blocked: **10** Points
- Clears Levine obstacles: **10** Points

### VII. Extra Resources

UNC Follow the Gap Video: https://youtu.be/ctTJHueaTcY
