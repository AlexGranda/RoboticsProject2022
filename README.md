# RoboticsProject2022

Students:
- Aida Alexandra Granda Cardenas
- Oscar Arturo Silva Castellanos

To run this project, we need to first open Coppelia using the following command:

```bash
~/apps/CoppeliaSim_Edu_V4_3_0_Ubuntu20_04/coppeliaSim.sh
```

Once Coppelia is open, we can load the final setup scene, which is called `RoboticsProject2022_track.ttt`, and can be found under the `scenes` folder. When the scene has been successfully loaded, we need to stablish a bridge with Coppelia to be able to communicate with the RoboMaster.

We have two main launcher files. The first one is just to establish our own bridge with Coppelia, which was created using the code provided by the professor and TAs in the `robomaster_ros` project. This bridge can be used with the following command:

```bash
ros2 launch RoboticsProject2022 main.launch name:=RM0 serial_number:=RM0
```

As can be observed from the command, the robot is called `RM0`. Once the connection has been stablished, we can run the following command in another tab to make the RoboMaster already placed over the line follow it:

```bash
ros2 launch RoboticsProject2022 project.launch.py name:=RM0 serial_number:=RM0
```

Some of the experimental results of processing images obtained with the camera of the RoboMaster can be found in the file called `RoboticsProject2022.ipynb`, that can be found in the main directory of the project.

We have recorded evidence of some successful attempts, you can find them in the following links:

- [First successful attempt](https://www.youtube.com/watch?v=_C33DVb1cUs&ab_channel=AlexandraGranda)
- [First visualization of real time images](https://www.youtube.com/watch?v=qm8XMwvHkl0&ab_channel=AlexandraGranda)
- [Attempt using Hough Lines](https://www.youtube.com/watch?v=s-vljXK2NkY&ab_channel=AlexandraGranda)
- [Final successful attempt in a closed path](https://www.youtube.com/watch?v=79j3lSkaLQE&ab_channel=AlexandraGranda)

