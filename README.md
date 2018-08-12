# fanlearn

### To install the project from Github you need:
1)Install GIT:
```
sudo apt-get install git
```
2)Create new workspace:
```
mkdir fanlearn_ws
cd fanlearn_ws/
mkdir src
cd src
git clone https://github.com/l1va/fanlearn.git
```
There are 4 packages:

fl_description - description

fl_compvis - computer vision

fl_gazebo - gazebo robot package

fl_hardware - interactions between ROS and fanuc electric drive system

### Project Description
NAME: FanLearn (Fanuc CR-7iA/L+ Reinforcement Learning)
EQUIPMENT: Fanuc manipulator  + camera  +  LEGO like bricks (XXL size)

Task description
There is a brick on the table <-- this is the goal position.
Human takes this brick and puts somewhere on the table <-- this is the start position
Camera gives us a picture, from that picture we define a brick's position

The manipulator should move the brick from the start position to the goal position.
The manipulator doesn't use any finger-like gripper, it uses a T-like pusher. 

Basically we have 2 environments: 2D environment of the table's surface and 6D environment of manipulator's joint-space. 

The first step
Solve 2D environment task using RL methods (RL stands for Reinforcement Learning)
Write a dynamic model of the manipulator: move from point A to point B or something like that (hardcode)
Combine RL solution and dynamic model

The next (optional) step (more bricks)
Do the same from the first step, but there are 2 or 3 bricks on the table

The next step (more learning)
Solve 2D environment task using RL methods  <-- the same as on previous step
Find a dynamic model of the manipulator: move from point A to point B <-- using RL method
Combine these 2 solutions together

