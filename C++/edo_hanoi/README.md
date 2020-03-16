# e.DO Hanoi Challenge

## Setup

Clone this repository into `~/catkin_ws/src` and run `catkin build` from `~/catkin_ws`.

Read the README.md in `edo_hanoi/src/neural` for the setup of the neural network.

## Running

Start the simulation and the services node using the following commands:

```
roslaunch edo_hanoi hanoi_simulation.launch
roslaunch edo_hanoi hanoi_services.launch
```

Both launch files accept the argument `sim` (default: `sim:=true`) to choose between simulation and the real robot. Use the `tokens` argument for the hanoi simulation to set how many tokens (hanoi pieces) will be used.

To run the Hanoi exercise, launch the detector and the client node:

```
python2 ~/catkin_ws/src/edo_hanoi/src/hanoi_state_server.py
roslaunch edo_hanoi hanoi_client.launch 
```

The client launch file also accepts the `sim` and the `tokens` argument. It also has an argument for the edo2 robot, so when working with edo2, set `edo2:=true` to use the correct configuration data. If you want to start the real robot without using the camera (but the default hanoi start instead), pass `use_camera:=false`.

In the hanoi state server you can switch between the neural network implementation and the clustering by changing the variable defined at the top. The hanoi state servers expects you to put a white background behind the board when taking the picture (do this when you start the client node, and wait until the node asks you to continue).

All arguments passed to the nodes with the same name have to have the same value.

