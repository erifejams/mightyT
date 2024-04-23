# MightThymio
# Using 
• Python <br>
• CoppeliaSim and the simulated MyT <br>
• ROS2 <br>
• Linux <br>


# Tasks
1. Write an open-loop controller that moves the MyT in such a way that it follows an "8" trajectory. Test it in the default empty scene. 

2. Using the wall scene file, write a controller to move the MyT straight ahead. Note that this scene rotates randomly the wall every time it is reset. We assume the robot is heading toward a wall somewhere in front of it; the wall is not necessarily orthogonal to the direction the MyT is originally pointing to. Write the controller in such a way that the MyT moves straight ahead until it is close to the wall (without hitting it), then turns in place in such a way to face the wall as precisely as possible (i.e., the robot's x-axis should be orthogonal to the wall). To sense the wall orientation once you are close to it, you should use proximity sensors. Feel free to define a convenient distance threshold at which you decide to stop. 

3. Using the controller built-in task 2, once the MyT arrives close to the wall, it should then turn in such a way that it is facing opposite to the wall, then move and stop in such a way that the robot is as close as possible to a point that is 2 meters away from the wall. Note that the proximity sensors don't have such a long-range, so at some point, you'll have to rely on odometry. <br>

# To run 
## Terminal #1:
### (EVERYTIME CHANGES ARE MADE TO THE PACKAGE CD TO THE ~/dev_ws/ folder): <br>
•	cd ~/dev_ws <br>
• colcon build <br>
•	source ~/dev_ws/install/setup.bash <br>

## Terminal #2:
### to get the coppelia sim running
cd ~/apps/CoppeliaSim_Edu_V4_4_0_rev0_Ubuntu20_04<br>
bash coppeliaSim.sh <br>
load the scene that can be found in Scene/final.ttm <br>
press the start button to start the simulation <br>


## Terminal #3:
source ~/dev_ws/install/setup.bash <br>
ros2 launch thymioid main.launch device:="tcp:host=localhost;port=33333" simulation:=True name:=thymio0


## Terminal #4:
### To run the compulsary.launch.xml (for task 3):
if you want to run the other task, copy the line below, but just change the name of the launch file as seen in the launch file folder e.g compulsary.launch.xml -> controller2.launch.xml  <br>  <br>
• source ~/dev_ws/install/setup.bash <br>
• ros2 launch thymio_example compulsary.launch.xml thymio_name:=thymio0 <br>


## Terminal #5:
#### to see the visualisation and camera from the robot pov
ros2 topic list (to see the topics of the robot - thymio0) <br>
rqt


