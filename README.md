# Final Assignment (Robotics Practicals)
This is the readme file about the final assignment of Robotics Practicals by **Cristian Meo** and **Joris Verhagen**
This assignment concerns controlling a prius car, by making it able to detect obstacles and people, to avoid objects and stop when people are detected.

The assignment is split in 3 main packages: OpenCV_Solution, PCL_Solution and Control_Solution.

## SETUP

In order to run the simulation, a catkin workspace must be created. Inside the src folder, me41025_simulator folder and group21 folder must be cloned. 

Once both folders are inside the src folder, go to catkin_ws directory and build the code:

From the catkin_ws folder:

`source devel/setup.bash`<br>
`catkin_make`


## How to Launch the simulation

Once the build has been done, in order to launch the simulation, do:

`roslaunch control_solution solution.launch`

Don't forget to do source `devel/setup.bash`, because if the environment is not setted properly, the launcher won't be found. Moreover, in order to see if you are actually in the right environment, try to right the launch command once a bit and to complete both names of pkg and node by using tab button. If you are in the right environment, the both pkg and node names should be compiled automatically once you type the first letters. 

## Simulation 
The simulation can be runned on different scenarios. Look at the manual to see which ones are available and how to modify it. On our simulation the standard one will be used. 
Once the simulation runs, the car should follow the path defined by cones and stop when people are detected. In order to be sure, try to run the simulation more than one time, sometimes it doesn't work. Probably due to simulation instabilities. 

## OpenCV_solution 
the opencv_solution package subscribes from the 
`/prius/front_camera/image_raw`
topic from the prius simulation. It processes the received images using the OpenCV library and publishes the detections as a ROS topic. The opencv_solution is responsible for the detection of people. Every time a person is detected, it will put a rectangle on the person in the simulation. In the control_solution this will ensure that the car brakes when it is in front of a person.

## PCL_solution
the pcl_solution package subscribes from the
`/point_cloud`
topic from the prius simulation. The pcl_solution is respensabile of detecting objects. It creates a point cloud and detect all kind of objects around the car, in the case of this assignment, predominantly cones but the ambulance is also detected as an object. From the raw point cloud, first the floor surface is removed, subsequentially the 'clouds' of points are detected and grouped when they exceed a certain threshold. From the pcl_solution, you receive a vector of 'objects' which represents cones. The properties of these objects are (with relevance to our scenario), an x-location, an y-location, and a size.
Everytime an object is detected, it is shown a green block on the point cloud screen. Moreover, it sends detections to the control node. 


## Control_Solution
Taking people and objects detections from OpenCV and PCL, it is able to control the car avoiding obstacles and people. 
The used algorithm is the following:

* If no object from the pcl is detected that is within a range of 4 meters and is actually in front of the car, the car goes straight with full throttle. <br>
* If obastacles are on the left, the car goes to the right with full throttle and viceversa. <br>
* If a person is detected, the car stops with full braking capacity and the steering and throttle action are set to zero. The car will remain in that position indefinitely.



