<!--
# How to Upload an Image

To upload an image in Markdown, you can use the following syntax:

```markdown
![Alt text](url_to_image)/ or path
-->

# Husky Virtual Commissioning

Husky – ROS2 Navigation with Omniverse ISAAC SIM
The project aims to streamline the commissioning process of robotic systems, particularly focusing on the Husky platform, through the integration of ROS2 for navigation and Omniverse ISAAC SIM for simulation. 

Husky is a rugged, outdoor-ready unmanned ground vehicle (UGV), suitable for research and rapid prototyping applications.

## 1. ROS2 for Navigation:
- ROS2 (Robot Operating System 2) provides a robust framework for controlling robotic systems.
- Utilizing ROS2 allows for seamless integration with various sensors, actuators, and software modules.
- Navigation capabilities within ROS2 enable the Husky platform to autonomously navigate its environment, avoiding obstacles and reaching designated destinations.

## 2. Omniverse ISAAC SIM Simulation:
- NVIDIA's Omniverse platform offers a powerful simulation environment with ISAAC SIM, tailored for robotics and AI applications.
- ISAAC SIM provides high-fidelity simulation, allowing realistic testing of robotic behaviors and interactions with virtual environments.
- The virtual commissioning process in Omniverse ISAAC SIM enables comprehensive testing and validation of Husky's functionalities in diverse scenarios before deployment in the real world.

## 3. Integration:
- Seamless integration between ROS2 and Omniverse ISAAC SIM facilitates data exchange and communication between the navigation system and the simulation environment.
- ROS2 nodes communicate with the simulated Husky model in ISAAC SIM, enabling real-time control and feedback loop during virtual commissioning.
- This integration enables developers and engineers to iteratively refine and optimize the Husky's navigation algorithms and behaviors in a risk-free virtual environment.

## Steps

### Step 1: Workstation, Installation, Running
We need a workstation with NVIDIA RTX GPU and the latest versions of both NVIDIA Isaac Sim and ROS 2 Humble installed.

Follow this tutorial to download ROS2 Humble and its dependencies. 

Here you can edit the bashrc so that ros2 is automatically sourced every time you open a terminal, instead of manually sourcing it every time you start a new shell session.

Follow this tutorial to download Isaac Sim. 

First, ensure that the device is compatible and then download the Omniverse launcher. This will be the hub where all the Omniverse features can be accessed, including Isaac Sim.

For the following steps, we will be following the Husky Demo tutorial with minor edits to correct the bash and python scripts: [Husky Demo Tutorial](https://github.com/NVIDIA-AI-IOT/husky_demo/blob/main/README.md)

### Step 2: Clone the Demo
Clone the demo to try it on the workstation using the following script:

```console
git clone https://github.com/NVIDIA-AI-IOT/husky_demo.git
cd husky_demo
```

### Step 2.5: Edit the Scripts

#### Edit `husky_demo.sh`

1. Open `husky_demo.sh`
2. Find line 121, which should be a part of `if ! $HIL_DEMO ; then` section.
3. Edit by adding `-d` in front of `$ISAAC_ROS_PATH` this ensures that the workspace is created using a correct directory.
4. The corrected line should look like this:

```console
gnome-terminal  --title="Isaac ROS terminal" -- sh -c "bash -c \"scripts/run_dev.sh -d $ISAAC_ROS_PATH; exec bash\""
```

### Edit `huksy_isaac_sim.py`

1. Open `huksy_isaac_sim.py`.
2. Find lines 160-161 under the `callback_description` method.
3. Correct the path to meshes and mesh accessories to ensure that husky robots will have meshes in Isaac Sim.

   - **Incorrect version:** This will have `install/share` which is a wrong path:
     ```python
     path_meshes = os.path.join(os.getcwd(), "isaac_ros", "install", "share", "husky_description", "meshes")
     path_mesh_accessories = os.path.join(os.getcwd(), "isaac_ros", "install", "share", "husky_isaac_sim", "meshes")
     ```

   - **Correct version:** This will have `install/share/husky_description` and `install/share/husky_isaac_sim`:
     ```python
     path_meshes = os.path.join(os.getcwd(), "isaac_ros", "install", "husky_description", "share", "husky_description", "meshes")
     path_mesh_accessories = os.path.join(os.getcwd(), "isaac_ros", "install", "husky_isaac_sim", "share", "husky_isaac_sim", "meshes")
     ```

### Step 3

After running the script, use the following command to start Isaac Sim and open a new terminal with Docker built to use Isaac ROS and all packages needed for the demo:

```sh
./husky_demo.sh
```

After you run this, a new terminal should appear called Isaac Ros Terminal. Check that you are in the correct directory by comparing the contents of `isaac_ros` in the `husky_demo` folder to the workspace directory by running the following command:

```sh
ls
```

The output should most likely contain `src` and `.gitignore`.

### Step 3.5

Next, we need to use Colcon to build and compile the packages in our workspace. If there are new `install` and `build` folders inside your `husky_demo` folder, remove them:

```sh
rm -rf install/
rm -rf build/
```

Next, run the following command:

```sh
colcon build --packages-up-to nvblox_rviz_plugin husky_isaac_sim husky_description xacro
```

### Step 4

Again, in Isaac Ros Terminal, which might be renamed now to colcon build, run the command:

```sh
source install/setup.bash
```

If this doesn’t work, close Isaac Sim and run `./husky_demo.sh` again. Remember that every time you run `./husky_demo.sh` you need to be inside the `husky_demo` directory:

```sh
cd husky_demo
./husky_demo.sh
```

Once the new terminal appears, source the `install/setup.bash`:

```sh
source install/setup.bash
```

### Step 5

Next, wait for Isaac Sim to fully load. You should see the warehouse environment loaded, and the terminal where Isaac Sim is running should output: “Robot Loader Start.” After this, you can run the following command to launch `rviz` to visualize the husky and to upload husky on Isaac Sim:

```sh
ros2 launch husky_isaac_sim robot_display.launch.py
```

If ROS2 doesn’t work, you could try sourcing it again:

```sh
source /opt/ros/humble/setup.bash
source install/setup.bash
```

**Note:** If there is an error in `rviz`, you can change the frame from `odom` to `base_link`. If you get an error when uploading Husky on Isaac Sim and you can’t see it, try running the `./husky_demo.sh` again. Then it will execute the launch file.
