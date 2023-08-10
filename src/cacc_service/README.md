# CACC Service 

This ros2 package consists of three ros2 nodes, a dummy OBU node acting as a publisher, and a CACC service and VehicleDynamics node acting both as a subscriber and publisher.

It simulates a OBU-CACC-VehicleDynamics loop for a set time of 20 seconds. Then saves the plot in the workspace folder of this package.

1. OBU node `obu_sensor_node.py` publishes to 'obu_topic' to which the CACC service node `cacc_service_node.py` subscribes. 

2. The CACC service node computes input u and publishes to 'acceleration_topic' to which the Vehicle node `vehicle_dynamics.py` subscribes.

3. The Vehicle node computes the next vehicle state, and sends it to CACC service node again. 

4. OBU node publishes data again, CACC computes input u again, etc.

A plot is shown during the execution.

A plot figure is saved in the ros2 workspace where this package was executed.

# Installation

### 1. Create ros2 workspace

First create your workspace folder with a src folder: 

```
mkdir local_ws
mkdir local_ws/src
```

Install possible dependencies and generate folders inside with (change 'humble' to your ros2 distro):

```
rosdep install -i --from-path src --rosdistro humble -y
```

Clone this repository into local_ws/src so you have to following structure:

local_ws/src/cacc_service/

Also clone or include the `cacc_interfaces` package inside the workspace, that this package depends on.

### 2. Build package

- install dependencies inside root of local_ws/:

```
rosdep install -i --from-path src --rosdistro humble -y
```

- build package with colcon

```
colcon build --packages-select cacc_service
```

### 3. Run nodes 

- in three new terminals, in each, source inside local_ws/:

```
  source install/setup.bash
```
- run each node in its own terminal with

1. Run Vehicle model first:
```
ros2 run cacc_service vehicle_dynamics
```
2. Run CACC Service:
```
ros2 run cacc_service cacc_service
```

3. Run OBU sensor:
```
ros2 run cacc_service obu_sensor
```

Logging should appear, the simulation will run for around 20 seconds.

If OBU runs first, data will be published but catched by no one.


### Development

After changing code, run steps 2. and 3. again. If adding dependencies, update your package.xml.
