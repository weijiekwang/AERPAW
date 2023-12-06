# Wireless Signal Localization with an Autonomous UAV on AERPAW

This experiment implements a wireless signal localization task on the AERPAW testbed, for the [AERPAW AFAR Challenge](https://aerpaw.org/aerpaw-afar-challenge/).

To reproduce this experiment on AERPAW, you will need to have [created an account on AERPAW that is part of a project](https://sites.google.com/ncsu.edu/aerpaw-wiki/aerpaw-user-manual/2-experiment-lifecycle-workflows/2-1-account-creation-and-authentication?authuser=0), and you will need to have [set up SSH keys in your account](https://sites.google.com/ncsu.edu/aerpaw-wiki/aerpaw-user-manual/2-experiment-lifecycle-workflows/2-2-account-authorization-and-roles?authuser=0#h.b7zx5bsw18bh). You will also need to have installed [a VPN client](https://openvpn.net/) and [QGroundControl](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html).


## Background

In this experiment, we are tasked with designing a trajectory for an autonomous UAV (drone), in order to localize an unmanned ground vehicle (UGV, rover) that is transmitting a wireless signal.

The rover will be randomly positions in the green bounding rectangle, and the drone may fly within the blue bounding rectangle: 

<img src="https://github.com/weijiekwang/AERPAW/assets/2316553/625757ef-d285-4b47-9bcb-11da0387e64d" width="400px">

Our work will be "scored" on: the estimated position of the rover at 3 minutes into the UAV flight, and the estimated position of the rover at 10 minutes into the UAV’s flight.

Our approach is in three parts:

* **Part 1**: Quickly approximate the rover position. To do this, we traverse the bottom edge of the bounding rectangle, and record the longitude at which the strongest signal is observed. Then, we traverse the left edge of the bounding rectangle, and record the latitude at which the strongest signal is observed. This latitude, longitude pair is our initial estimate of the rover position.
* **Part 2**: If our initial estimate suggests that the rover may be in or near the regions of its bounding rectangle that are *outside* the drone's bounding rectangle, we use an "OOB" procedure to estimate the distance of the rover from the edge of the drone's bounding rectangle. The drone flies to the current best estimate of the rover position, then moves 100m away from the rover (perpendicular to the bounding rectangle) and records measurements of signal strength along the way. It uses these measurements to estimate how far outside the drone's bounding rectangle the rover is located, and set either the latitude or longitude accordingly.
* **Part 3**: The drone flies to waypoints suggested by a [Bayesian Optimization](https://github.com/bayesian-optimization/BayesianOptimization) to train a [Gaussian Process](https://scikit-learn.org/stable/modules/gaussian_process.html), and measures signal strength at each waypoint in order to improve its estimate of the rover position. Here, we are trying to maximize signal strength as a function of latitude, longitude. Bayesian optimization is well suited for optimizing a black box function (does not assume a functional form), when it is expensive to sample the function at any given point, and when the output of the function may be noisy. ("Sampling the function at a given point" in this context, means flying the drone to that point and measuring the signal strength there!)

## Results

## Run my experiment

### Create an experiment with a UAV and UGV resource

First, log in to the [AERPAW Experiment Portal](https://user-web-portal.aerpaw.ncsu.edu/). Click on "Projects" in the navigation bar, and find the project that you are a member of; click on it to open the project overview. Click on the "Create" button in the "Experiments" section. Give your project a "Name" and a "Description" and then click "Save".

From the experiment page, you will add members (optional) and resources (required) to the experiment: 

<image src="https://github.com/weijiekwang/AERPAW/assets/123581716/fe7d3675-abc3-4895-ad4f-63b43d5e9aec" width="200px">

If multiple users will require access to the experiment, you should add them at this stage. Next to "Members", click "Update" and add any additional project members that will need access.

Next, click on the "Update" button next to "Targeted Resources". On this page, you will add two resources to the experiment: first select LPN1 (this will be the UAV, which is assumed to be the first vehicle in the rest of the instructions) and then select SPN-Android (this will be the UGV, which is assumed to be the second vehicle in the rest of the instructions):

<image src="https://github.com/weijiekwang/AERPAW/assets/123581716/0acd0690-26f2-4f40-a85b-89f7379c6bc2" width="800px">

Click "Save". Then, you will modify each of the nodes by clicking "Modify" - 

<image src="https://github.com/weijiekwang/AERPAW/assets/123581716/234e6b93-b6df-4a4c-8721-59778790359a" width="800px">

* for node 1 (LPN1), change the "Node Vehicle" property to "vehicle_uav" and click "Save"
* for node 2 (SPN-Android), change the "Node Vehicle" property to "vehicle_ugv" and click "Save"

<image src="https://github.com/weijiekwang/AERPAW/assets/123581716/d347737f-0e95-4218-a504-90190205a1d5" width="200px">
<image src="https://github.com/weijiekwang/AERPAW/assets/123581716/42cfb511-da48-4b5c-ba7b-e7af987bf942" width="200px">

Click "Back to Experiment", then "Initialize Development".

<image src="https://github.com/weijiekwang/AERPAW/assets/123581716/5229775b-ca15-4e9d-8b33-6cb9bb8113df" width="800px">

 Wait until you receive an email notification that the experiment is ready to use.


### Prepare UAV and UGV resources for an experiment

On the rover E-VM, set up the radio transmitter:

```
cd /root/Profiles/ProfileScripts/Radio 
cp Samples/startGNURadio-ChannelSounder-TX.sh startRadio.sh 
```

Set up the GPS logger on the rover:

```
cd /root/Profiles/ProfileScripts/Vehicle
cp Samples/startGPSLogger.sh Vehicle/startVehicle.sh
```

Open the overall experiment script for editing:

```
vim /root/startExperiment.sh # or whatever text editor
```
and un-comment the `./Vehicle/startVehicle.sh` and `./Radio/startRadio.sh` lines. Save and close.

On the drone E-VM, set up the radio transmitter:

```
cd /root/Profiles/ProfileScripts/Radio 
cp Samples/startGNURadio-ChannelSounder-RX.sh startRadio.sh 
```

Set up the rover search on the drone:

```
cd /root/Profiles/ProfileScripts/Vehicle
cp Samples/startRoverSearch.sh Vehicle/startVehicle.sh
```

Open the overall experiment script for editing:

```
sudo vim /root/startExperiment.sh # or whatever text editor you use
```

and un-comment the `./Vehicle/startVehicle.sh` and `./Radio/startRadio.sh lines`. Save and close.

### Run baseline rover search experiment

### Run our rover search experiment

On the drone E-VM, install dependencies for rover search:

```
python3 -m pip install --target=/root/Profiles/vehicle_control/RoverSearch bayesian-optimization
python3 -m pip install --target=/root/Profiles/vehicle_control/RoverSearch geopy
```

Transfer over the latest rover search code:

```
wget https://raw.githubusercontent.com/weijiekwang/AERPAW/main/rover_search.py -O /root/Profiles/vehicle_control/RoverSearch/rover_search.py
```

To start the experiment,

* in the OEO console, run `2 start_experiment`
* in QGroundControl, arm the rover (Vehicle 2) and move it to the desired location (within its bounding rectangle). Wait for it to reach its position.
* in the OEO console, run `1 start_experiment`
* in QGroundControl, arm the drone (Vehicle 1)

#### Drone flight pattern

After takeoff, the drone will fly to the SE corner of the bounding rectangle:

<img src="https://github.com/weijiekwang/AERPAW/assets/2316553/fdf9675e-c307-4846-8a3b-ce4e2145e381" width="400px"/>

then to the SW bound, and then to the NW: 

<img src="https://github.com/weijiekwang/AERPAW/assets/2316553/b42d82ec-7d82-40a5-88f1-07b3dda9c4ac" width="400px"/>


Then, it will fly to various waypoints within its bounding rectangle. Many of the waypoints will be very near to the rover, and the trajectory may appear somewhat random:

<img src="https://github.com/weijiekwang/AERPAW/assets/2316553/8480b5ce-1aa3-4bac-bc18-53bc71ea4517" width="400px"/>


It may also occasionally fly to more distant waypoints, including the corners of its bounding rectangle.

The rover search code will regularly print estimate of the rover position to `stdout` on the drone, e.g. 

```
Position estimate:  35.7288639 -78.6977849 0:09:46.246165
```
where the values are latitude, longitude, and search time. 

(Note that if the drone is not armed already when the code begins to run, so that it has to wait for the drone to be armed, the search time that is printed will not be accurate.)

After a ten-minute search time, the drone will land at its home position.

## Notes

### References
