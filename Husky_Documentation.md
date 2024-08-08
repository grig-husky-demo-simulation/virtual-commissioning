<a name="br1"></a> 

Husky Virtual Commissioning

**Husky – ROS2 Navigation with Omniverse ISAAC SIM**

The project aims to streamline the commissioning process of robotic systems,

particularly focusing on the Husky platform, through the integration of ROS2 for

navigation and Omniverse ISAAC SIM for simulation.

1



<a name="br2"></a> 

**STEP 1**

***Workstation, Installation, Running***

We need a workstation with NVIDIA RTX GPU and the latest versions of both [NVIDIA](https://developer.nvidia.com/isaac-sim)

[Isaac](https://developer.nvidia.com/isaac-sim)[ ](https://developer.nvidia.com/isaac-sim)[Sim](https://developer.nvidia.com/isaac-sim)[ ](https://developer.nvidia.com/isaac-sim)and [ROS](https://docs.ros.org/en/humble/Installation.html)[ ](https://docs.ros.org/en/humble/Installation.html)[2](https://docs.ros.org/en/humble/Installation.html)[ ](https://docs.ros.org/en/humble/Installation.html)[Humble](https://docs.ros.org/en/humble/Installation.html)[ ](https://docs.ros.org/en/humble/Installation.html)installed.

Follow this tutorial to download ROS2 Humble and its dependencies.

\-

Here you can edit the bashrc so that ros2 is automatically sourced every time you

open a terminal, instead of manually sourcing it every time you start a new shell

session.

Follow this tutorial to download Isaac Sim.

\-

First you should make sure that the device is compatible and then download the

omniverse launcher. This will be the hub where all the omniverse features can be

accessed, including Isaac Sim.

For the following steps we will be following the Husky Demo tutorial with minor edits to

correct the bash and python scripts:

<https://github.com/NVIDIA-AI-IOT/husky_demo/blob/main/README.md>

**STEP 2**

Clone the demo to try it on workstation using the following script:

git clone <https://github.com/NVIDIA-AI-IOT/husky_demo.git>

cd husky\_demo

2



<a name="br3"></a> 

**Step 2.5**

Before we move to the next step we have found several errors in the scripts in the

husky\_demo folder.

1\. Edit husky\_demo.sh:

a. Open husky\_demo.sh

b. Find line 121, which should be a part of if ! $HIL\_DEMO ; then section.

c. Edit by adding -d in front of $ISAAC\_ROS\_PATH this ensures that the

workspace is created using a correct directory.

d. The corrected line should look like this: gnome-terminal

--title="Isaac ROS terminal" -- sh -c "bash -c

\"scripts/run\_dev.sh -d $ISAAC\_ROS\_PATH; exec bash\""

3



<a name="br4"></a> 

2\. Edit huksy\_isaac\_sim.py:

a. Open huksy\_isaac\_sim.py

b. Find lines 160-161 under callback\_description method.

c. Correct path to meshes and mesh accessories. This will ensure that husky

robots will have meshes in Isaac Sim.

d. Incorrect version will have install/share which is a wrong path:

path\_meshes = os.path.join(os.getcwd(), "isaac\_ros", "install", "share",

"husky\_description", "meshes")

path\_mesh\_accessories = os.path.join(os.getcwd(), "isaac\_ros", "install",

"share", "husky\_isaac\_sim", "meshes")

e. Correct version will have install/share/husky\_description and

install/share/husky\_isaac\_sim:

path\_meshes = os.path.join(os.getcwd(), "isaac\_ros", "install",

"husky\_description", "share", "husky\_description", "meshes")

path\_mesh\_accessories = os.path.join(os.getcwd(), "isaac\_ros", "install",

"husky\_isaac\_sim", "share", "husky\_isaac\_sim", "meshes")

4



<a name="br5"></a> 

**Step 3**

After running it, use the following command to start Isaac Sim and open a new terminal

with Docker built to use Isaac ROS and all packages needed for the demo:<sup>1</sup>

./husky\_demo.sh

After you run this a new terminal should appear called Isaac Ros Terminal. Check that

you are in a correct directory by comparing the contents of isaac\_ros in husky\_demo

folder to the workspace directory by running the following command: ls

The output should most likely contain src and .gitignore

**Step 3.5**

Next, we need to Colcon build to compile and build the packages in our workspace. If

there are new install and build folders inside of your husky\_demo folder remove it

rm -rf install/

rm -rf build/

1

This command will:

● Clone all repositories required

● Clone the Isaac ROS Docker image

● Build a new image

5



<a name="br6"></a> 

Next, run the following command:

colcon build --packages-up-to nvblox\_rviz\_plugin husky\_isaac\_sim

husky\_description xacro

**Step 4**

Again, in Isaac Ros Terminal, which might be renamed now to colcon build. Run the

command:

source install/setup.bash

If this doesn’t work. Close Isaac Sim and run ./husky\_demo.sh again. Remember that

every time you run ./husky\_demo.sh you need to be inside the husky\_demo directory.

cd husky\_demo

./husky\_demo.sh

Once the new terminal appears source teh install/setup.bash

source install/setup.bash

6



<a name="br7"></a> 

**Step 5 (main)**

Next, wait for Isaac Sim to fully load. You should see the warehouse environment loaded

and the terminal where Isaac Sim is running should output: “Robot Loader Start.” After

this you can run the following command to launch rviz to visualize the husky and to

upload husky on Isaac Sim:

ros2 launch husky\_isaac\_sim robot\_display.launch.py

If ROS2 doesn’t work you could try sourcing it.

source /opt/ros/humble/setup.bash

**Note:** If there is an error in rviz you can change the frame from odom to base\_link

Finally if the you get an error when uploading Husky on Isaac Sim and you can’t see it try

running the ./husky\_demo.sh

Then it will execute the launch ﬁle - This ROS 2 script launches all Isaac ROS packages

to localize the robot and start mapping and rviz to visualize the husky on map. The

script will also load a Husky on the environment and automatically set up cameras and

controllers.

7



<a name="br8"></a> 

**Step 6**

Next, open a new terminal, run the command to set up the driving for the Husky.

ros2 run teleop\_twist\_keyboard teleop\_twist\_keyboard

\-

\-

ros2 run: This is the command used to run a node in ROS

teleop\_twist\_keyboard: This is the name of the package that contains the

node for keyboard teleoperation.

\-

teleop\_twist\_keyboard: This is the speciﬁc node within the package that

you are running. It allows you to control the robot using keyboard inputs.

Figure 1: Husky robot in Omniverse ISAAC SIM environment (on the right) and the

driving control keyboard panel layout (on the left).

8



<a name="br9"></a> 

**Step 7**

Let's add a Lidar sensor to the robot.

1\. **Add the Lidar Sensor:**

○

○

Go to Create -> Isaac -> Sensors -> PhysX Lidar -> Rotating.

Drag the Lidar prim under /husky/sesor\_arch\_mount\_link and place

it in the same location as the top of the robot’s base

Zero out any displacement in the Transform fields inside the

Property tab to ensure proper alignment by translating the z-axis

to approximately -0.45.

○

2\. **Configure the Lidar Sensor:**

○

Inside the *RawUSDProperties* tab for the Lidar prim, set maxRange to

25 to ignore anything beyond 25 meters.

Check drawLines and drawPoints to visualize the Lidar scans and

points.

○

3\. **Test the Lidar Sensor:**

Press Play to see the Lidar come to life. Red lines indicate a hit,

○

green means no hit, and the color spectrum from green to yellow to

red is proportional to the distance of the object detected.

9



<a name="br10"></a> 

**Step 8**

Once the Lidar sensor is in place, we need to add the corresponding ROS 2 nodes

to stream the detection data to a ROS 2 topic (ROS2 Bridge).

1\. **Add Lidar OG Nodes:**

Use the following nodes to publish Lidar data to ROS 2:

●

**On Playback Tick Node:** Produces a tick when the simulation is "Playing".

Nodes receiving ticks from this node will execute their compute functions

every simulation step.

●

**Isaac Read Lidar Beam Node:** Retrieves information about the Lidar and

data. For inputs: LidarPrim, add the target to point to the Lidar sensor

added at /husky/sesor\_arch\_mount\_link/lidar.

●

●

**ROS 2 Publish Laser Scan:** Publishes laser scan data. Type /laser\_scan

into the Topic Name field.

**Isaac Read Simulation Time:** Uses simulation time to timestamp the

/laser\_scan messages.

10



<a name="br11"></a> 

**Step 9**

**Verify ROS 2 Connections**

● Press Play to start ticking the graph and the physics simulation.

● Open a separate ROS 2-sourced terminal and check that the associated ROS 2

topics exist with: ros2 topic list

○ /laser\_scan should be listed in addition to /rosout.

● Open RViz 2 by typing in rviz2 on the command line if not already open.

○ Inside RViz 2, add a LaserScan type to visualize.

○ Ensure the Topic that the laser scan is listening to matches the topic name

inside the ROS 2 Publish Laser Scan node (should be sim\_lidar),

and the ﬁxed frame matches the frame\_id inside the ROS 2 Publish

Laser Scan node (should be laser\_scan).

○ Increase the size of dots inside

Laser Scan to 0.08 m and adjust

the Grid parameters to ﬁt in the

mapping of the environment.

○ Add Image type to visualize the

image of

/front/stereo\_camera/rgb/depth

11



<a name="br12"></a> 

**Step 10**

Now we can use the following approach to make sure that the lidar sensor data is

correctly integrated:

**Checking for Missing Transforms:** Ensure that the lidar sensor frame is properly deﬁned

and connected in the TF tree. As this is missing, and we see a white silhouette of the

robot model, we add a static transform between base\_link and the lidar sensor

frame.

ros2 run tf2\_ros static\_transform\_publisher 0 0 0 0 0 0 1

base\_link sim\_lidar

ros2 run tf2\_ros static\_transform\_publisher 0 0 0 0 0 0 1 odom base\_link

12



<a name="br13"></a> 

13



<a name="br14"></a> 

**Step 11**

This part will elaborate on how to create an occupancy map in Omniverse Isaac Sim via

the extension to generate a 2-dimensional occupancy map for an environment.

1\. In the Occupancy Map Generator UI set the origin to an empty location in the

stage.

2\. You will see a wireframe rectangle appear in the stage showing the bounds of the

area used to create the map

3\. The center of this rectangle must be in an unoccupied space

4\. Select the Warehouse prim in the stage. In the Occupancy Map extension, click

on BOUND SELECTION.

5\. For the Upper Bound, set the Z height and also modify the parameters to cover

the whole environment including a bit outside of the boundaries as well.

6\. Press CALCULATE followed by VISUALIZE IMAGE. A window showing the map will

appear.

14



<a name="br15"></a> 

7\. Visualization Window:

a. Occupied/Freespace/Unknown Colors: These colors are used in the final

generated image

b. Rotation: Rotates the output image by the specified amount

c. Coordinate Type: Selects the type of configuration output for the

generated occupancy map

d. Occupancy Map:

Pressing RE-GENERATE

IMAGE will create a new image

and display the updated

configuration output

Press the Save Image button and select the location where you wish to save the image.

The final stored image will look like above.

15

