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

# Update

2024/10/24: fix the gravity coefficient; add useful tools in `utils/tools.py`.

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