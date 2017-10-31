# Synopsis

Here used DUO3D, JY901BT and ST LSM6DSL/M plus LIS2MDL IMU devices associated with sensor calibration, sensor filtering and fusion algorithms, and sensor fusion with visual SLAM, e.g. ORBSlam. 

#  IMU Project File Description
## "IMU_Unity_Server" Project

- IMU_DUO3D.cs
  DUO3D IMU Unity server script, used to receive the orientation data from DUO3D client
- IMU_JY901BT.cs
  JY901BT IMU Unity server script, used to receive the orientation data from JY901BT client
## "IMUJY901_python_client" Project
It's the JY901BT IMU client, used to send Gyo data to IMU Unity server
