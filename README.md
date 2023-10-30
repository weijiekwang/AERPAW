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

Install dependencies for rover search:

```
python3 -m pip install --target=/root/Profiles/vehicle_control/RoverSearch bayesian-optimization
python3 -m pip install --target=/root/Profiles/vehicle_control/RoverSearch geopy
```

Transfer over the latest rover search code:

```
wget https://raw.githubusercontent.com/weijiekwang/AERPAW/main/rover_search.py -O /root/Profiles/vehicle_control/RoverSearch/rover_search.py
```

Open the overall experiment script for editing:

```
sudo vim /root/startExperiment.sh # or whatever text editor you use
```

and un-comment the `./Vehicle/startVehicle.sh` and `./Radio/startRadio.sh lines`. Save and close.

### Run baseline rover search experiment

### Run our rover search experiment

## Notes

### References
