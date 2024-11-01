# Default-Course-Project
Default Course Project of CS4278/CS5478 Intelligent Robots: Algorithms and Systems

![Scene](imgs/scene.png)


In this project, the task is to:

1. Navigate the Mobot to the drawer
2. Pick up the mug
3. Place it into the drawer

We provide an example control code for the robot in `simulation/main.py`.

# Requirement

You should implement the navigation and motion planning algorithms by yourself to accomplish the task.

# Rubic

When the total score of the project is 100 points:

### Navigation (45%)

You should navigate the robot to go from the starting point to any place after the narrow passage at the right room. When you arrive the goal region, the `main.py` function will print a successful signal. 

You will get corresponding points based on your total navigation driving distance:

45% for total distance ≤ 18;
40% for 18 < total distance ≤ 22;
30% for 22 < total distance ≤ 25;
20% for 25 < total distance ≤ 28
10% for 28 < total distance ≤ 32
0% for total distance > 32.

you should provide at least three successful navigation videos to show the robustness of your algorithm. We will use the slowest video for grading.


### Manipulation (45%)

#### motion planning algorithm (30%)

You should implement a motion planning algorithm for the provided stretch robot arm end-effector to go to any given point in the task space, including: 

path planning (10%);
collision avoidance (10%);
inverse kinematics calculation (10%).

You should provide a video to show that your robot arm can reach different given positions in the task space.

#### Successfully pick and place the mug (15%)

Successfully pick and place the target mug into the drawer. When your algorithm achieves this, the `main.py` function will print a successful signal.

10%: if only pick up the mug;
15%: pick and place the mug.

You should provide at least three successful pick and place (or only pick) videos to show the robustness of your algorithms.

### Project Materials (10%)

Clear and readable project report (4%)
Runable codebase (4%)
Clear video results (2%)

### Bonus (10%)

5%: if you implement trajectory planning for the robot arm (including inverse dynamics)

5% if you can pick and place a randomly initialized mug. You can start random initialization of the mug at [here](). You should provide at least three successful videos for this part.

# Update

2024/11/01: add rubic and helper functions

2024/10/25: add attach and detach functions for grasping in `simulation/main.py`. Now you can use this to grasp and release the mug. Try it!

2024/10/24: add useful tools in `utils/tools.py`.

2024/10/22: fix the gravity coefficient; add useful tools in `utils/tools.py`.

2024/10/21: add keyboard control example code in `simulation/main.py`.


# Installation

1. Create a new Anaconda environment: `conda create -n mm python=3.8`

2. `conda activate mm`

3. `pip3 install pybullet numpy`

4. `git clone https://github.com/NUS-LinS-Lab/Mobile-Manipulation.git`

# Run

`python simulation/main.py`

# References

PyBullet Documentation: https://pybullet.org/wordpress/index.php/forum-2/.

You can also use ChatGPT or Cursor to get the API of PyBullet.