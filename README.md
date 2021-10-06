Firstly, i was using ubuntu 18 and ros melodic, I installed the ubuntu 20, ros2 foxy, and SVL simulator.

In SVL simulator, I couldn't connect the lgsvl_bridge I dont know why. After some package installing and removing, i could connect svl to lgsvl_bridge and i can read the topics and msg types. Then i created 2 nodes with ros2. One node reads the velocity from /lgsvl/gnss_odom and publish the thrust to /lgsvl/vehicle_control_cmd. I used PID controller, and tuned it by using SVL simulator. In sample code, the target velocity is 25m/s. Steady state error occured and about 0.1 m/s.

![Optional Text](../main/images/ss1.png)
![Optional Text](../main/images/ss2.png)
![Optional Text](../main/images/ss3.png)
