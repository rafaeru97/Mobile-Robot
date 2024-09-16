## Mobile Robot Navigation System
This master's thesis presents a complete mobile robot navigation system, utilizing the A* algorithm for path planning and SLAM techniques for building a map of the environment. The system is based on the Raspberry Pi platform, which coordinates the operation of sensors such as the MPU-6050 gyroscope, motor encoders, and a distance sensor. Real-time data processing enables the robot to effectively avoid obstacles and navigate in dynamic environments.

The system offers flexibility and easy scalability. The Python-based program allows for the integration of additional features, while the grid map structure and A* algorithm ensure high precision in determining optimal routes. Moreover, the system supports communication via a graphical interface accessible through a web browser, simplifying interaction with the robot and enabling real-time monitoring of its status.

While the system functions as intended, there is potential for further optimization, particularly in terms of computational performance. Certain computations could be migrated to C++ to speed up the program's execution.

Verification tests conducted in real-world conditions confirmed the system's accuracy, with results indicating the possibility for further development towards more advanced robotic applications.

## Project Gallery
Here are some images showcasing the mobile robot navigation system in action:

<p align="center">
  <img src="https://i.imgur.com/LvPiTOh.png">
</p>

<p align="center">
  <img src="https://i.imgur.com/qOoyDM9.png">
</p>

<p align="center">
  <img src="https://i.imgur.com/4T1sIJb.png">
</p>

<p align="center">
  <img src="https://i.imgur.com/l3zMYjI.png">
</p>

<p align="center">
  <img src="https://i.imgur.com/TS9tR7P.png">
</p>