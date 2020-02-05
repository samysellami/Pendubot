# BLDC_Stand_Simulator
-
HOW VIEW THE STAND IN GAZEBO?
-
Open the terminal

```bash
cd "your workspace"/src/
```
```bash
git clone <Future_GitHub_Link>
```
```bash
cd ..
```
```bash
catkin_make
```
```bash
roslaunch bldc_stand_sim model.launch 

```

-
HOW TO RUN POSITION CONTROL OF THE STAND IN GAZEBO?
-
Open the terminal

```bash
cd "your workspace"/src/
```
```bash
git clone <Future_GitHub_Link>
```
```bash
cd ..
```
```bash
catkin_make
```
```bash

roslaunch bldc_stand_sim control_position.launch & rqt

```
```bash

Go to Plugins-->Message Publisher--> Choose topics command/ --> Put some law
(in order to close push (ctrl+C), fg and (ctrl+C))
```

HOW TO RUN TORQUE CONTROL OF THE STAND IN GAZEBO?
-
Open the terminal

```bash
cd "your workspace"/src/
```
```bash
git clone <Future_GitHub_Link>
```
```bash
cd ..
```
```bash
catkin_make
```
```bash

roslaunch bldc_stand_sim control_torque.launch & rqt

```

```bash

Go to Plugins-->Message Publisher--> Choose topics command/ --> Put some law
(in order to close push (ctrl+C), fg and (ctrl+C))

```


```bash
