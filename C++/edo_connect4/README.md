# e.DO Connect 4 Challenge

## Prerequisites

You will need:

```bash
sudo apt-get install portaudio19-dev
```

And a Python2 virtualenv:
```bash
sudo pip2 install -U virtualenv
virtualenv --system-site-packages -p python2 ~/venv2
source ~/venv2/bin/activate
```
 
Where once activated you install:
```bash
pip install -r ~/catkin_ws/src/edo_connect4/requirements.txt
```
If you have PyAudio errors just check these instructions out, [link](https://stackoverflow.com/questions/20023131/cannot-install-pyaudio-gcc-error).

## Setup

Clone this repository into `~/catkin_ws/src` and run `catkin build` from `~/catkin_ws`.

## Running

Start the simulation and the services node using the following commands:

```
roslaunch edo_connect4 connect4_simulation.launch
roslaunch edo_connect4 connect4_services.launch
roslaunch edo_connect4 dialogflow.launch
```

Both launch files accept the argument `sim` (default: `sim:=true`) to choose between simulation and the real robot. The services launch file also starts the vision node. This node will display the board detection, you'll have to close this window for the vision service to start.

To run the Connect4 exercise, launch the client node:

```
roslaunch edo_connect4 connect4_client.launch 
```

The client launch file also accepts the `sim` argument. It also has an argument for the edo2 robot, so when working with edo2, set `edo2:=true` to use the correct configuration data.

You can pass the `use_camera` argument to the client node to disable/enable the use of the camera for board detection. Instead of using the camera, you will then have to input your move into the console.

All arguments passed to the nodes with the same name have to have the same value.

Here is an example of the client launch file, given some common real-robot args:
```
roslaunch edo_connect4 connect4_client.launch sim:=false use_camera:=true speak:=true player_name:="Red Cobras" edo2:=false dice:=false
```

## Playing against the AI only

If you just want to play against the AI without using the robot, you can run the interactive program:

```
rosrun edo_connect4 edo_connect4_interactive
```

Who starts will be chosen at random. You can input your move as a number representing the column, 0 being the left-most column and 6 the right-most.

## Two robots battle scenario

Simply launch all launch files with "battle" in their name to start a demo where two robots play against each other in simulation.

