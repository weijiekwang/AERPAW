import asyncio
import math
import datetime
import csv

from typing import List
from struct import unpack
from argparse import ArgumentParser

from aerpawlib.runner import StateMachine
from aerpawlib.vehicle import Vehicle, Drone
from aerpawlib.runner import state, timed_state
from aerpawlib.util import Coordinate, VectorNED
from aerpawlib.safetyChecker import SafetyCheckerClient

from radio_power import RadioEmitter

# step size between each radio measurement
MIN_STEP_SIZE = 0.1 # never move less than this much in a step
MAX_STEP_SIZE = 100 # never move more than this much in a step
STEP_SIZE = 40  # when going forward - how far, in meters

# move along four cardinal directions
WEST = 270 # azimuth in degrees
EAST = 90
NORTH = 360
SOUTH = 180
DEG_TOLERANCE = 10
HEADINGS_LIST = [WEST, NORTH, EAST, SOUTH]

SEARCH_ALTITUDE = 30 # in meters

class RoverSearch(StateMachine):
    last_measurement = float("-inf")
    best_measurement = float("-inf")
    best_pos: Coordinate = None
    start_time = None
    search_time = None

    total_steps = 0
    steps_this_heading = 0

    # Keep track of current heading index
    heading_idx = 0 

    # Initially, bounds are None
    # over time, we will discover bounds and add them here
    bounds = {'w': None, 'e': None, 'n': None, 's': None}

    def initialize_args(self, extra_args: List[str]):
        """Parse arguments passed to vehicle script"""
        # Default output CSV file for search data
        defaultFile = "ROVER_SEARCH_DATA_%s.csv" % datetime.datetime.now().strftime(
            "%Y-%m-%d_%H:%M:%S"
        )

        parser = ArgumentParser()
        parser.add_argument("--safety_checker_ip", help="ip of the safety checker server")
        parser.add_argument("--safety_checker_port", help="port of the safety checker server")
        parser.add_argument(
            "--fake_radio",
            help="Launch the accompanying radio power script as a fake radio source",
            required=False,
            default=False,
        )
        parser.add_argument(
            "--log",
            help="File to record kml of search path and RSSI values",
            required=False,
            default=defaultFile,
        )
        parser.add_argument(
            "--save_csv",
            help="Whether to save a kml file with the search path and RSSI values",
            default=True,
        )
        parser.add_argument(
            "--search_time",
            help="How long in minutes to search for the rover",
            required=False,
            default=10,
            type=int,
        )

        args = parser.parse_args(args=extra_args)

        self.fake_radio = args.fake_radio
        self.safety_checker = SafetyCheckerClient(args.safety_checker_ip, args.safety_checker_port)
        self.log_file = open(args.log, "w+")
        self.save_csv = args.save_csv
        self.search_time = datetime.timedelta(minutes=args.search_time)

        # Create a CSV writer object if we are saving data
        if self.save_csv:
            self.csv_writer = csv.writer(self.log_file)
            self.csv_writer.writerow(["timestamp","longitude", "latitude", "altitude", "RSSI"])

    @state(name="start", first=True)
    async def start(self, vehicle: Drone):
        # record the start time of the search
        self.start_time = datetime.datetime.now()
        # create a fake radio if the arg was passed in
        if self.fake_radio:
            # use the radio emitter script as a fake radio for testing
            self.radio_emitter = RadioEmitter()
            print("Using a fake radio")
        print("Taking off")
        await vehicle.takeoff(SEARCH_ALTITUDE)
        print("Took off")

        # Let's start flying west first (we're taking of in the east)
        turning = asyncio.ensure_future(vehicle.set_heading( HEADINGS_LIST[self.heading_idx]  ))

        # wait for vehicle to finish turning
        while not turning.done():
            await asyncio.sleep(0.1)

        return "go_forward"

    @state(name="go_forward")
    async def go_forward(self, vehicle: Vehicle):
        # go forward and continually log the vehicle's position
        print("Moving Forward")

        self.total_steps = self.total_steps + 1
        self.steps_this_heading = self.steps_this_heading + 1
        
        # use the current vehicle heading to move forward
        heading = vehicle.heading
        heading_rad = heading * 2 * math.pi / 360

        # check if we have crossed a bound, if so, modify step size
        bound_dist = 1
        # we may have discovered a new bound, so update
        if (  self.bounds['n']  and ( (NORTH % 360) <= heading <= ((NORTH + DEG_TOLERANCE) % 360) ) or
             ( (NORTH - DEG_TOLERANCE) <= heading <= (NORTH) )  ):
            bound_dist = abs(vehicle.position.lat - self.bounds['n'])

        elif ( self.bounds['e'] and (EAST - DEG_TOLERANCE) <= heading <= (EAST + DEG_TOLERANCE) ) :
            print(f"Heading was {heading} so discovered new bound moving E")
            bound_dist = abs(vehicle.position.lon - self.bounds['e'])

        elif (  self.bounds['s'] and (SOUTH - DEG_TOLERANCE) <= heading <= (SOUTH + DEG_TOLERANCE) ) :
            print(f"Heading was {heading} so discovered new bound moving S")
            bound_dist = abs(vehicle.position.lat - self.bounds['s'])

        elif ( self.bounds['w'] and (WEST - DEG_TOLERANCE) <= heading <= (WEST + DEG_TOLERANCE) ) :
            print(f"Heading was {heading} so discovered new bound moving W")
            bound_dist = abs(vehicle.position.lon - self.bounds['w'])


        # decrease step size with total steps
        # increase step size if we keep going in same direction
        decay_factor = (20/(20+self.total_steps))
        change_factor = ((10+self.steps_this_heading)/10)
        bound_factor = 1
        computed_step_size = decay_factor*change_factor*bound_factor*STEP_SIZE
        step_size = computed_step_size        
        
        step_size = max(step_size, MIN_STEP_SIZE)
        step_size = min(step_size, MAX_STEP_SIZE)

        #print(f"Heading: {heading}")
        print(f"Distance to bound: {bound_dist}")
        print(f"Step tracker: {self.steps_this_heading}, {self.total_steps}, {step_size}")

        move_vector = VectorNED(
            step_size * math.cos(heading_rad), step_size * math.sin(heading_rad), 0
        )

        # ensure the next location is inside the geofence
        cur_pos = vehicle.position
        next_pos = vehicle.position + move_vector
        (valid_waypoint, msg) = self.safety_checker.validateWaypointCommand(
            cur_pos, next_pos
        )
        
        # if the next location violates the geofence turn 90 degrees
        if not valid_waypoint:
            print("Can't go there:")
            print(msg)
            return "turn_right_90"
                
        # otherwise move forward to the next location
        moving = asyncio.ensure_future(
            vehicle.goto_coordinates(vehicle.position + move_vector)
        )

        # wait until the vehicle is done moving
        while not moving.done():
            await asyncio.sleep(0.1)

        await moving
        return "take_measurement"

    @state(name="turn_right_90")
    async def turn_right_90(self, vehicle: Drone):
        # turn right before moving forward again
        print("turning")
        heading = vehicle.heading

        # turn - go to next heading in list
        self.heading_idx = (self.heading_idx + 1) % 4
        new_heading = HEADINGS_LIST[self.heading_idx]

        # we may have discovered a new bound, so update
        if ( ( (NORTH % 360) <= heading <= ((NORTH + DEG_TOLERANCE) % 360) ) or
             ( (NORTH - DEG_TOLERANCE) <= heading <= (NORTH) )  ):
            print(f"Heading was {heading} so discovered new bound moving N")
            if ( (self.bounds['n'] is None) or (vehicle.position.lat < self.bounds['n']) ): 
                print(f"Updating bound from {self.bounds['n']} to {vehicle.position.lat}" )
                self.bounds['n'] = vehicle.position.lat

        elif ( (EAST - DEG_TOLERANCE) <= heading <= (EAST + DEG_TOLERANCE) ) :
            print(f"Heading was {heading} so discovered new bound moving E")
            if ( (self.bounds['e'] is None) or (vehicle.position.lon < self.bounds['e']) ): 
                print(f"Updating bound from {self.bounds['e']} to {vehicle.position.lon}" )
                self.bounds['e'] = vehicle.position.lon

        elif ( (SOUTH - DEG_TOLERANCE) <= heading <= (SOUTH + DEG_TOLERANCE) ) :
            print(f"Heading was {heading} so discovered new bound moving S")
            if ( (self.bounds['s'] is None) or (vehicle.position.lat > self.bounds['s']) ): 
                print(f"Updating bound from {self.bounds['s']} to {vehicle.position.lat}" )
                self.bounds['s'] = vehicle.position.lat

        elif ( (WEST - DEG_TOLERANCE) <= heading <= (WEST + DEG_TOLERANCE) ) :
            print(f"Heading was {heading} so discovered new bound moving W")
            if ( (self.bounds['w'] is None) or (vehicle.position.lon > self.bounds['w']) ): 
                print(f"Updating bound from {self.bounds['w']} to {vehicle.position.lon}" )
                self.bounds['w'] = vehicle.position.lon

        self.steps_this_heading = 0
        turning = asyncio.ensure_future(vehicle.set_heading(new_heading))

        # wait for vehicle to finish turning
        while not turning.done():
            await asyncio.sleep(0.1)

        await turning
        return "go_forward"

    @timed_state(name="take_measurement", duration=1)
    async def take_measurement(self, vehicle: Drone):
        # Take a radio power measurement and decide to move forward or turn

        # Take a fake radio measurement if configured
        if self.fake_radio:
            measurement = self.radio_emitter.get_power(vehicle.position)
            print(f"Fake measurement: {measurement}")
        # Otherwise take a real measurement
        else:
            # Open data buffer
            f = open("/root/Power", "rb")
            # unpack binary reading into a float
            measurement_from_file = unpack("<f", f.read(4))
            measurement = measurement_from_file[0]
            # close the data buffer
            f.close()
            print(f"Real measurement: {measurement}")

        # If the radio measurement has increased, keep moving forward
        if measurement >= self.last_measurement:
            next = "go_forward"
        # Otherwise turn
        else:
            next = "turn_right_90"
        self.last_measurement = measurement

        # Track the best location and measurement
        if measurement > self.best_measurement:
            self.best_measurement = measurement
            self.best_pos = vehicle.position

        # save the current position and measurement if logging to file
        if self.save_csv:
            position = vehicle.position
            self.csv_writer.writerow(
                [datetime.datetime.now() - self.start_time, position.lon, position.lat, position.alt, measurement]
            )

        # If the search time has ended, end the script
        if datetime.datetime.now() - self.start_time > self.search_time:
            next = "end"

        return next

    @state(name="end")
    async def end(self, vehicle: Drone):
        # Return vehicle to start and land it
        print(
            f"Search time of {self.search_time} minutes has elapsed. Returning to launch!"
        )
        print(
            f"Best rover location estimate {self.best_pos.lat, self.best_pos.lon} with measurement {self.best_measurement} after {datetime.datetime.now()-self.start_time} minutes"
        )
        home_coords = Coordinate(
            vehicle.home_coords.lat, vehicle.home_coords.lon, vehicle.position.alt
        )
        await vehicle.goto_coordinates(home_coords)
        await vehicle.land()
        print("Done!")
