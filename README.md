# Wireless Signal Localization with an Autonomous UAV on AERPAW

## Background

## Results

## Run my experiment

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
* in QGroundControl, arm the rover and move it to the desired location (within its bounding rectangle). Wait for it to reach its position.
* in the OEO console, run `1 start_experiment`
* in QGroundControl, arm the drone (Vechile 1)

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
