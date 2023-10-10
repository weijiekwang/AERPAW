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

from bayes_opt import BayesianOptimization, UtilityFunction, SequentialDomainReductionTransformer

BOUND_NE={'lon':-78.69621514941473, 'lat':35.72931030026633}
BOUND_NW={'lon':-78.69953825817279, 'lat':35.72931030026633}
BOUND_SE={'lon':-78.69621514941473, 'lat':35.72688213193035}
BOUND_SW={'lon':-78.69953825817279, 'lat':35.72688213193035}


MAX_LON = BOUND_NE['lon']
MIN_LON = BOUND_NW['lon']
MAX_LAT = BOUND_NE['lat']
MIN_LAT = BOUND_SE['lat']

SEARCH_ALTITUDE = 30 # in meters

STATE_TAKEOFF    = 0
STATE_LON_SEARCH = 1
STATE_LAT_SEARCH = 2
STATE_STEADY     = 3

def argmax(x):
    return max(range(len(x)), key=lambda i: x[i])

class RoverSearch(StateMachine):
    last_measurement = float("-inf")
    best_measurement = float("-inf")
    best_pos: Coordinate = None
    start_time = None
    search_time = None

    probe_state = STATE_TAKEOFF
    measurement_list = []
    # start at the SE bound
    next_waypoint = {'lat': BOUND_SE['lat'], 'lon': BOUND_SE['lon']}
    start_best_pos = {'lat': None, 'lon': None}
    
    # See https://github.com/bayesian-optimization/BayesianOptimization/blob/master/examples/advanced-tour.ipynb
    # Note: using sequential domain reduction to improve convergence time
    # https://github.com/bayesian-optimization/BayesianOptimization/blob/master/examples/domain_reduction.ipynb
    optimizer = BayesianOptimization(
      f=None,
      pbounds={'lat': (MIN_LAT, MAX_LAT), 'lon': (MIN_LON, MAX_LON)},
      verbose=0,
      random_state=1,
      allow_duplicate_points=True
    )

    # Note: change this utility function to manage the 
    # exploration/exploitation tradeoff
    # see: https://github.com/bayesian-optimization/BayesianOptimization/blob/master/examples/exploitation_vs_exploration.ipynb
    utility = UtilityFunction(kind="ucb", kappa=2.5, xi=0.0)

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
            self.csv_writer.writerow(["timestamp","longitude", "latitude", "altitude", "RSSI", "best_lat", "best_lon"])

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

        return "probe"


    @state(name="register")
    async def register(self, vehicle: Drone):
        print("Now in register")
        # register most recent measurements
        for m in self.measurement_list:
            print("Registering: ", m ) 
            self.optimizer.register(params={'lat': m['lat'], 'lon': m['lon']}, target=m['power'])

        # see if we are in initial lon/lat search, or in steady state
        if self.probe_state == STATE_TAKEOFF:
            # we're at BOUND_SE now, start longitude search
            self.measurement_list = []
            self.next_waypoint = {'lat': BOUND_SW['lat'], 'lon': BOUND_SW['lon']}
            self.probe_state = STATE_LON_SEARCH
            print("Setting next state to LON_SEARCH")
            return "probe"
        elif self.probe_state == STATE_LON_SEARCH:
            # get best longitude
            idx_max_lon = argmax([m['power'] for m in self.measurement_list])
            self.start_best_pos['lon'] = self.measurement_list[idx_max_lon]['lon']
            # now start latitude search
            self.measurement_list = []
            self.next_waypoint = {'lat': BOUND_NW['lat'], 'lon': BOUND_NW['lon']}
            self.probe_state = STATE_LAT_SEARCH
            print("Setting next state to LAT_SEARCH")
            return "probe"
        elif self.probe_state == STATE_LAT_SEARCH:
            # get best latitude
            idx_max_lat = argmax([m['power'] for m in self.measurement_list])
            self.start_best_pos['lat'] = self.measurement_list[idx_max_lat]['lat']
            # now start steady state - first go to best position
            self.measurement_list = []
            self.next_waypoint = {'lat': self.start_best_pos['lat'], 'lon': self.start_best_pos['lon']}
            self.probe_state = STATE_STEADY
            print("Setting next state to STEADY")
            return "probe"

        elif self.probe_state == STATE_STEADY:
            # Track the best location and measurement
            max_estimate = self.optimizer.max
            if max_estimate['target'] > self.best_measurement:
                self.best_measurement = max_estimate['target']
                self.best_pos = Coordinate(max_estimate['params']['lat'], max_estimate['params']['lon'], SEARCH_ALTITUDE)

            # save the positions and measurements if logging to file
            if self.save_csv:
                for m in self.measurement_list:
                    position = Coordinate(m['lat'], m['lon'], SEARCH_ALTITUDE)
                    self.csv_writer.writerow(
                        [datetime.datetime.now() - self.start_time, m.lon, m.lat, m.alt, m['power'], self.best_pos.lat,self.best_pos.lon]
                    )

            # reset measurement list
            self.measurement_list = []
            return "suggest"

    @state(name="suggest")
    async def suggest(self, vehicle: Drone):

        # suggest the next waypoint to visit
        self.next_waypoint = self.optimizer.suggest(self.utility)

        # go to next waypoint
        return "probe"

    @state(name="probe")
    async def probe(self, vehicle: Drone):

        # stop if search time is over
        if datetime.datetime.now() - self.start_time > self.search_time:
            return "end"

        print("Going to: ", self.next_waypoint['lat'], self.next_waypoint['lon'], SEARCH_ALTITUDE )
        # go to the waypoint, probe along the way
        next_pos =  Coordinate(self.next_waypoint['lat'], self.next_waypoint['lon'], SEARCH_ALTITUDE)
        (valid_waypoint, msg) = self.safety_checker.validateWaypointCommand(
            vehicle.position, next_pos
        )
        if valid_waypoint:
            moving = asyncio.ensure_future(
                vehicle.goto_coordinates(next_pos)
            )
            while not moving.done():

                # Take a fake radio measurement if configured
                if self.fake_radio:
                    measurement = self.radio_emitter.get_power(vehicle.position)
                    #print(f"Fake measurement: {measurement}")
                # Otherwise take a real measurement
                else:
                    # Open data buffer
                    f = open("/root/Power", "rb")
                    # unpack binary reading into a float
                    measurement_from_file = unpack("<f", f.read(4))
                    measurement = measurement_from_file[0]
                    # close the data buffer
                    f.close()
                    #print(f"Real measurement: {measurement}")

                self.measurement_list.append( 
                    {'lat': vehicle.position.lat, 'lon': vehicle.position.lon, 'power': measurement} 
                    )
                print("Measured: ",
                    {'lat': vehicle.position.lat, 'lon': vehicle.position.lon, 'power': measurement} 
                    )
                
                await asyncio.sleep(0.1)
            print("Now at: ", vehicle.position.lat, vehicle.position.lon, measurement )

                
        return "register"


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
