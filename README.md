HuskyVirtualCommissioning![](Aspose.Words.74ec9231-fe7f-4dee-b744-c0806cefcb2c.001.jpeg)

**Husky – ROS2 Navigation with Omniverse ISAACSIM**

The project aims to streamline the commissioning process of robotic systems, particularly focusing on the Husky platform, through the integration of ROS2 for navigation and Omniverse ISAAC SIMfor simulation.

1![](Aspose.Words.74ec9231-fe7f-4dee-b744-c0806cefcb2c.002.png)

**STEP 1![ref1]**

***Workstation, Installation, Running***

We need a workstation with NVIDIARTXGPUand the latest versions of both [NVIDIA Isaac Sim ](https://developer.nvidia.com/isaac-sim)and [ROS2 Humble ](https://docs.ros.org/en/humble/Installation.html)installed.

Follow this tutorial to download ROS2 Humble and its dependencies.

- Here you can edit the bashrc so that ros2 is automatically sourced every time you open a terminal, instead of manually sourcing it every time you start a new shell session.

Follow this tutorial to download Isaac Sim.

- First you should make sure that the device is compatible and then download the omniverse launcher. This willbe the hub where all the omniverse features can be accessed, including Isaac Sim.![](Aspose.Words.74ec9231-fe7f-4dee-b744-c0806cefcb2c.004.png)

For the following steps we willbe following the Husky Demo tutorial with minor edits to correct the bash and python scripts: <https://github.com/NVIDIA-AI-IOT/husky_demo/blob/main/README.md>

**STEP 2**

Clone the demo to try it on workstation using the following script:

git clone <https://github.com/NVIDIA-AI-IOT/husky_demo.git>![](Aspose.Words.74ec9231-fe7f-4dee-b744-c0806cefcb2c.005.png)

cd husky\_demo

2![ref2]

**Step 2.5![ref1]**

Before we move to the next step we have found several errors in the scripts in the husky\_demo folder.

1. Edit husky\_demo.sh:
1. Open husky\_demo.sh
1. Find line 121, which should be a part of if ! $HIL\_DEMO ; then section.
1. Edit by adding -d in front of $ISAAC\_ROS\_PATH this ensures that the workspace is created using a correct directory.
1. The corrected line should look like this: gnome-terminal --title="Isaac ROS terminal" -- sh -c "bash -c \"scripts/run\_dev.sh -d $ISAAC\_ROS\_PATH; exec bash\""

![](Aspose.Words.74ec9231-fe7f-4dee-b744-c0806cefcb2c.007.jpeg)

3![ref2]


2. Edit huksy\_isaac\_sim.py:![ref1]
1. Open huksy\_isaac\_sim.py
1. Find lines 160-161 under callback\_description method.
1. Correct path to meshes and mesh accessories. This willensure that husky robots willhave meshes in Isaac Sim.
1. Incorrect version willhave install/share which is a wrong path:

path\_meshes = os.path.join(os.getcwd(), "isaac\_ros", "install", "share", ![ref3]"husky\_description", "meshes")

path\_mesh\_accessories = os.path.join(os.getcwd(), "isaac\_ros", "install", "share", "husky\_isaac\_sim", "meshes")

5. Correct version willhave install/share/husky\_description and install/share/husky\_isaac\_sim:

path\_meshes = os.path.join(os.getcwd(), "isaac\_ros", "install", ![ref3]"husky\_description", "share", "husky\_description", "meshes") path\_mesh\_accessories = os.path.join(os.getcwd(), "isaac\_ros", "install", "husky\_isaac\_sim", "share", "husky\_isaac\_sim", "meshes")

![](Aspose.Words.74ec9231-fe7f-4dee-b744-c0806cefcb2c.009.jpeg)

4![ref2]

**Step 3**![ref1]**

After running it,use the following command to start Isaac Sim and open a new terminal with Docker built to use Isaac ROSand all packages needed for the demo:[^1]

./husky\_demo.sh![ref4]

After you run this a new terminal should appear called Isaac Ros Terminal. Check that you are in a correct directory by comparing the contents of isaac\_ros in husky\_demo folder to the workspace directory by running the following command: ls

The output should most likely contain src and .gitignore

**Step 3.5**

Next, we need to Colcon build to compile and build the packages in our workspace. If there are new install and build folders inside of your husky\_demo folder remove it

rm -rf install/ rm -rf build/![](Aspose.Words.74ec9231-fe7f-4dee-b744-c0806cefcb2c.011.png)


Next, run the following command:![ref1]

colcon build --packages-up-to nvblox\_rviz\_plugin husky\_isaac\_sim husky\_description xacro![](Aspose.Words.74ec9231-fe7f-4dee-b744-c0806cefcb2c.012.png)

**Step 4**

Again, in Isaac Ros Terminal, which might be renamed now to colcon build. Run the command:

source install/setup.bash![ref5]

Ifthis doesn’twork. Close Isaac Sim and run ./husky\_demo.sh again. Remember that every time you run ./husky\_demo.sh you need to be inside the husky\_demo directory.

cd husky\_demo ./husky\_demo.sh![](Aspose.Words.74ec9231-fe7f-4dee-b744-c0806cefcb2c.014.png)

Once the new terminal appears source teh install/setup.bash

source install/setup.bash![ref5]

6![ref2]

**Step 5 (main)![ref1]**

Next, wait for Isaac Sim to fully load. You should see the warehouse environment loaded and the terminal where Isaac Sim is running should output: “Robot Loader Start.”After this you can run the following command to launch rviz to visualize the husky and to upload husky on Isaac Sim:

ros2 launch husky\_isaac\_sim robot\_display.launch.py![ref4]

IfROS2 doesn’twork you could try sourcing it.

source /opt/ros/humble/setup.bash![](Aspose.Words.74ec9231-fe7f-4dee-b744-c0806cefcb2c.015.png)

**Note:** Ifthere is an error in rviz you can change the frame from odom to base\_link

Finally if the you get an error when uploading Husky on Isaac Sim and you can’tsee it try running the ./husky\_demo.sh

Then it willexecute the launch file -This ROS2 script launches all Isaac ROSpackages to localize the robot and start mapping and rviz to visualize the husky on map. The script willalso load a Husky on the environment and automatically set up cameras and controllers.

7![ref2]

**Step 6![ref1]**

Next, open a new terminal, run the command to set up the driving for the Husky.

ros2 run teleop\_twist\_keyboard teleop\_twist\_keyboard![](Aspose.Words.74ec9231-fe7f-4dee-b744-c0806cefcb2c.016.png)

- ros2 run: This is the command used to run a node in ROS
- teleop\_twist\_keyboard: This is the name of the package that contains the node for keyboard teleoperation.![](Aspose.Words.74ec9231-fe7f-4dee-b744-c0806cefcb2c.017.jpeg)
- teleop\_twist\_keyboard: This is the specific node within the package that you are running. It allows you to control the robot using keyboard inputs.

![](Aspose.Words.74ec9231-fe7f-4dee-b744-c0806cefcb2c.018.jpeg)

Figure 1: Husky robot in Omniverse ISAAC SIM environment (on the right) and the driving control keyboard panel layout (on the left).

8![ref2]

|<p>Let's 1.</p><p>2\.</p><p>3\.</p>|<p>**Step 7![ref1]**</p><p>add a Lidar sensor to the robot.</p><p>**Add the Lidar Sensor:**</p><p>- Go to Create -> Isaac -> Sensors -> PhysX Lidar -> Rotating.</p><p>- Drag the Lidar prim under /husky/sesor\_arch\_mount\_link and place</p><p>&emsp;it in the same location as the top of the robot’s base</p><p>- Zero out any displacement in the Transform fields inside the Property tab to ensure proper alignment by translating the z-axis to approximately -0.45.</p><p>**Configure the Lidar Sensor:**</p><p>- Inside the *RawUSDProperties* tab for the Lidar prim, set maxRange to 25 to ignore anything beyond 25 meters.</p><p>- Check drawLines and drawPoints to visualize the Lidar scans and points.</p><p>**Test the Lidar Sensor:**</p><p>- Press Play to see the Lidar come to life. Red lines indicate a hit, green means no hit, and the color spectrum from green to yellow to red is proportional to the distance of the object detected.</p>|
| -: | - |
||9|
|||
**Step 8![ref1]**

Once the Lidar sensor is in place, we need to add the corresponding ROS 2 nodes to stream the detection data to a ROS 2 topic (ROS2 Bridge).

1\. **Add Lidar OG Nodes:**

Use the following nodes to publish Lidar data to ROS 2:

- **On Playback Tick Node:** Produces a tick when the simulation is "Playing". Nodes receiving ticks from this node will execute their compute functions every simulation step.
- **Isaac Read Lidar Beam Node:** Retrieves information about the Lidar and data. For inputs: LidarPrim, add the target to point to the Lidar sensor added at /husky/sesor\_arch\_mount\_link/lidar.
- **ROS 2 Publish Laser Scan:** Publishes laser scan data. Type /laser\_scan

  into the Topic Name field.

- **Isaac Read Simulation Time:** Uses simulation time to timestamp the /laser\_scan messages.

![](Aspose.Words.74ec9231-fe7f-4dee-b744-c0806cefcb2c.019.png)

10![ref2]

**Step 9![ref1]**

**Verify ROS 2 Connections**

- Press Play to start ticking the graph and the physics simulation.
- Open a separate ROS2-sourced terminal and check that the associated ROS2 topics exist with: ros2 topic list
  - /laser\_scan should be listed in addition to /rosout.
- Open RViz2 by typing in rviz2 on the command line if not already open.
- Inside RViz2, add a LaserScan type to visualize.
- Ensure the Topic that the laser scan is listening to matches the topic name inside the ROS 2 Publish Laser Scan node (should be sim\_lidar), and the fixed frame matches the frame\_id inside the ROS 2 Publish Laser Scan node (should be laser\_scan).
- Increase the size of dots inside Laser Scan to 0.08 m and adjust ![](Aspose.Words.74ec9231-fe7f-4dee-b744-c0806cefcb2c.020.png)the Grid parameters to fit in the mapping of the environment. 
- Add Image type to visualize the image of /front/stereo\_camera/rgb/depth 

11![ref2]


Now we can use the following approach to make sure that the lidar sensor data is correctly integrated:

**Checking for Missing Transforms:** Ensure that the lidar sensor frame is properly defined and connected in the TF tree. As this is missing, and we see a white silhouette of the robot model, we add a static transform between base\_link and the lidar sensor

frame.

ros2 run tf2\_ros static\_transform\_publisher 0 0 0 0 0 0 1 base\_link sim\_lidar

ros2 run tf2\_ros static\_transform\_publisher 0 0 0 0 0 0 1 odom base\_link

![](Aspose.Words.74ec9231-fe7f-4dee-b744-c0806cefcb2c.021.jpeg)

12![ref2]

![](Aspose.Words.74ec9231-fe7f-4dee-b744-c0806cefcb2c.022.png)

13![ref2]![ref1]


This part will elaborate on how to create an occupancy map in Omniverse Isaac Sim via the extension to generate a 2-dimensional occupancy map for an environment.

1. In the Occupancy Map Generator UI set the origin to an empty location in the stage.
1. You will see a wireframe rectangle appear in the stage showing the bounds of the area used to create the map
1. The center of this rectangle must be in an unoccupied space
1. Select the Warehouse prim in the stage. In the Occupancy Map extension, click on BOUND SELECTION.
1. For the Upper Bound, set the Z height and also modify the parameters to cover the whole environment including a bit outside of the boundaries as well.
1. Press CALCULATE followed by VISUALIZE IMAGE. A window showing the map will appear.![](Aspose.Words.74ec9231-fe7f-4dee-b744-c0806cefcb2c.023.jpeg)

14![ref2]

7. Visualization Window:![ref1]
1. Occupied/Freespace/Unknown Colors: These colors are used in the final generated image
1. Rotation: Rotates the output image by the specified amount
1. Coordinate Type: Selects the type of configuration output for the generated occupancy map![](Aspose.Words.74ec9231-fe7f-4dee-b744-c0806cefcb2c.024.jpeg)
   4. Occupancy Map:

Pressing RE-GENERATE IMAGE will create a new image and display the updated configuration output

Press the Save Image button and select the location where you wish to save the image. The final stored image will look like above.

15![ref2]![ref2]

[^1]: This command will:
- 
    Clone all repositories required
- 
    Clone the Isaac ROSDocker image
- 
    Build a new image

    5
[ref1]: Aspose.Words.74ec9231-fe7f-4dee-b744-c0806cefcb2c.003.png
[ref2]: Aspose.Words.74ec9231-fe7f-4dee-b744-c0806cefcb2c.006.png
[ref3]: Aspose.Words.74ec9231-fe7f-4dee-b744-c0806cefcb2c.008.png
[ref4]: Aspose.Words.74ec9231-fe7f-4dee-b744-c0806cefcb2c.010.png
[ref5]: Aspose.Words.74ec9231-fe7f-4dee-b744-c0806cefcb2c.013.png
