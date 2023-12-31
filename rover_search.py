import asyncio
import math
import datetime
import csv
import pickle

from typing import List
from struct import unpack
from argparse import ArgumentParser


from aerpawlib.runner import StateMachine
from aerpawlib.vehicle import Vehicle, Drone
from aerpawlib.runner import state, timed_state
from aerpawlib.util import Coordinate, VectorNED
from aerpawlib.safetyChecker import SafetyCheckerClient

from radio_power import RadioEmitter

import numpy as np
from bayes_opt import BayesianOptimization, UtilityFunction, SequentialDomainReductionTransformer
from sklearn.gaussian_process.kernels import Matern, WhiteKernel, RBF, RationalQuadratic, ExpSineSquared, DotProduct

import geopy
import geopy.distance

BOUND_NE={'lon':-78.69621514941473, 'lat':35.72931030026633}
BOUND_NW={'lon':-78.69953825817279, 'lat':35.72931030026633}
BOUND_SE={'lon':-78.69621514941473, 'lat':35.72688213193035}
BOUND_SW={'lon':-78.69953825817279, 'lat':35.72688213193035}


MAX_LON = BOUND_NE['lon']
MIN_LON = BOUND_NW['lon']
MAX_LAT = BOUND_NE['lat']
MIN_LAT = BOUND_SE['lat']

SEARCH_ALTITUDE = 50 # in meters

STATE_TAKEOFF    = 0
STATE_LON_SEARCH = 1
STATE_LAT_SEARCH = 2
STATE_STEADY     = 3
STATE_OOB_N      = 4
STATE_OOB_W      = 5

SIG_BOUND = 43
SIG_BOUND_LOW = 36
LAT_BOUND_SLACK = MAX_LAT - (MAX_LAT-MIN_LAT)*0.02
LON_BOUND_SLACK = MIN_LON + (MAX_LON-MIN_LON)*0.02

HEADING_SEQ_N = [-1, 180]
HEADING_SEQ_W = [-1, 90]

OOB_CYCLE = 5 # after every X intervals in steady state, go to OOB state again

def argmax(x):
    return max(range(len(x)), key=lambda i: x[i])

class RoverSearch(StateMachine):
    last_measurement = float("-inf")
    best_measurement = float("-inf")
    best_pos: Coordinate = None
    start_time = None
    search_time = None

    heading_seq_n_idx = 0
    heading_seq_w_idx = 0
    oob_cycle_idx = 0

    probe_state = STATE_TAKEOFF
    measurement_list = []
    # start at the SE bound
    next_waypoint = {'lat': BOUND_SE['lat'], 'lon': BOUND_SE['lon']}
    start_best_pos = {'lat': None, 'lon': None}
    oob_best_pos   = {'lat': None, 'lon': None}

    # See https://github.com/bayesian-optimization/BayesianOptimization/blob/master/examples/advanced-tour.ipynb
    # Note: using sequential domain reduction to improve convergence time
    # https://github.com/bayesian-optimization/BayesianOptimization/blob/master/examples/domain_reduction.ipynb
    optimizer = BayesianOptimization(
      f=None,
      pbounds={'lat': (MIN_LAT, MAX_LAT), 'lon': (MIN_LON, MAX_LON)},
      verbose=0,
      random_state=1,
      bounds_transformer = SequentialDomainReductionTransformer(minimum_window=0.001),
      allow_duplicate_points=True
    )

    # Note: change this utility function to manage the 
    # exploration/exploitation tradeoff
    # see: https://github.com/bayesian-optimization/BayesianOptimization/blob/master/examples/exploitation_vs_exploration.ipynb
    utility = UtilityFunction(kind="ucb", kappa=1)

    # set the kernel, alpha parameter
    kernel = Matern(length_scale = 1.0, nu = 1.5)  + WhiteKernel(noise_level=0.5)
    optimizer._gp.set_params(kernel = kernel)
    #optimizer._gp.set_params(alpha=1e-3)

    #with open('gaussian_process.pickle', 'wb') as handle:
    #    pickle.dump(optimizer._gp, handle)

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

        #print("Current state: ", self.probe_state)

        # register last value
        self.optimizer.register(params={'lat': vehicle.position.lat, 'lon': vehicle.position.lon}, target=self.measurement_list[-1]['power'])
        # register max value
        idx = np.argmax([m['power'] for m in self.measurement_list])
        self.optimizer.register(params={'lat': self.measurement_list[idx]['lat'], 'lon': self.measurement_list[idx]['lon']}, target=self.measurement_list[idx]['power'])
        # register a random sample of values
        #n_samples = int(np.min(5, len(self.measurement_list)/2))
        #for m in np.random.choice(self.measurement_list, size=n_samples, replace=False):
        #    self.optimizer.register(params={'lat': m['lat'], 'lon': m['lon']}, target=m['power'])
        # register all samples
        #for m in self.measurement_list:
        #    self.optimizer.register(params={'lat': m['lat'], 'lon': m['lon']}, target=m['power'])

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
            # set best position now - to save time
            self.best_pos = Coordinate(self.start_best_pos['lat'], self.start_best_pos['lon'], SEARCH_ALTITUDE)
            print("Position estimate: ", self.best_pos.lat, self.best_pos.lon, datetime.datetime.now() - self.start_time)
            # now start steady state - first go to best position
            self.measurement_list = []
            self.next_waypoint = {'lat': self.start_best_pos['lat'], 'lon': self.start_best_pos['lon']}

            # if we think the rover is out of bounds...
            if ( LAT_BOUND_SLACK <= self.start_best_pos['lat'] <=  MAX_LAT ) :
                self.probe_state = STATE_OOB_N
                print("Setting next state to STATE_OOB_N")

            elif (  MIN_LON <= self.start_best_pos['lon'] <= LON_BOUND_SLACK )  :
                self.probe_state = STATE_OOB_W
                print("Setting next state to STATE_OOB_W")

            # otherwise, go to steady state
            else:
                self.probe_state = STATE_STEADY
                print("Setting next state to STEADY")

            return "probe"

        elif self.probe_state == STATE_STEADY:

            # Track the best location and measurement
            max_estimate = self.optimizer.max
            if max_estimate['target'] > self.best_measurement:
                self.best_measurement = max_estimate['target']
                self.best_pos = Coordinate(max_estimate['params']['lat'], max_estimate['params']['lon'], SEARCH_ALTITUDE)
            if self.oob_best_pos['lon']:
                self.best_pos.lon =  self.oob_best_pos['lon']
            if self.oob_best_pos['lat']:
                self.best_pos.lat =  self.oob_best_pos['lat']
            print("Position estimate: ", self.best_pos.lat, self.best_pos.lon, datetime.datetime.now() - self.start_time)
            
            # save the positions and measurements if logging to file
            if self.save_csv:
                for m in self.measurement_list:
                    self.csv_writer.writerow(
                        [datetime.datetime.now() - self.start_time, m['lat'], m['lon'], SEARCH_ALTITUDE, m['power'], self.best_pos.lat,self.best_pos.lon]
                    )

            # reset measurement list
            self.measurement_list = []

            # keep track of whether it's time to repeat OOB search procedure
            self.oob_cycle_idx = (self.oob_cycle_idx + 1 ) % OOB_CYCLE
            #print("OOB cycle index", self.oob_cycle_idx)

            if (self.oob_cycle_idx+1==OOB_CYCLE) and  (LAT_BOUND_SLACK <= self.start_best_pos['lat'] <=  MAX_LAT ) :
                self.probe_state = STATE_OOB_N
                print("Setting next state to STATE_OOB_N")
                self.next_waypoint = {'lat': self.best_pos.lat, 'lon': self.best_pos.lon}
                return "probe"

            elif (self.oob_cycle_idx+1==OOB_CYCLE) and (  MIN_LON <= self.start_best_pos['lon'] <= LON_BOUND_SLACK )  :
                self.probe_state = STATE_OOB_W
                print("Setting next state to STATE_OOB_W")
                self.next_waypoint = {'lat': self.best_pos.lat, 'lon': self.best_pos.lon}
                return "probe"


            return "suggest"
    
        elif self.probe_state == STATE_OOB_N:
            #idx = np.argmax([m['power'] for m in self.measurement_list])
            #print("Best observation on this trip: ", self.measurement_list[idx]['lat'], self.measurement_list[idx]['lon'], self.measurement_list[idx]['power'] )
            if self.heading_seq_n_idx>=1:
                #print(self.measurement_list)
                meas = np.array( [d['power'] for d in self.measurement_list])
                idx = np.where(np.logical_and(meas>=SIG_BOUND_LOW, meas<=SIG_BOUND))[0]
                meas = np.array( [d['power'] for i, d in enumerate(self.measurement_list) if i in idx ])
                lats = np.array( [d['lat'] for i, d in enumerate(self.measurement_list) if i in idx ] )
                lons = np.array( [d['lon'] for i, d in enumerate(self.measurement_list) if i in idx ] )
                
                #dist = 465.1662158364831 + -9.655778240458593*meas
                dist = 492.21897944364474 +  -10.249809981197462*meas
                # estimated latitudes of rover
                est_lat = [ geopy.distance.distance(meters = d).destination(point=geopy.Point(la, lo), bearing=0).latitude for la, lo, d in zip(lats, lons, dist) ]
                if len(est_lat):
                    #print("Updating latitude from %f to %f based on %d measurements from %f to %f" % (self.best_pos.lat, np.median(est_lat), len(est_lat), np.min(meas), np.max(meas) ) ) 
                    self.best_pos.lat = np.median(est_lat)
                    self.oob_best_pos['lat'] = np.median(est_lat)
                print("Position estimate: ", self.best_pos.lat, self.best_pos.lon, datetime.datetime.now() - self.start_time)

            # reset measurement list
            self.measurement_list = []

            self.heading_seq_n_idx = self.heading_seq_n_idx+1

            #if (self.heading_seq_n_idx > len(HEADING_SEQ_N) -1 ) and (  MIN_LON <= self.start_best_pos['lon'] <= LON_BOUND_SLACK )  :
            #    self.probe_state = STATE_OOB_W
            #    print("Setting next state to STATE_OOB_W")
            # don't do OOB-W search if already did OOB-N search

            if (self.heading_seq_n_idx > len(HEADING_SEQ_N) -1 ):
                self.heading_seq_n_idx = 0
                self.probe_state = STATE_STEADY
                print("Setting next state to STEADY")

                return "suggest"
            
            return "suggest_seq"


        elif self.probe_state == STATE_OOB_W:

            #idx = np.argmax([m['power'] for m in self.measurement_list])
            #"Best observation on this trip: ", self.measurement_list[idx]['lat'], self.measurement_list[idx]['lon'], self.measurement_list[idx]['power'] )

            if self.heading_seq_w_idx>=1:
                #print(self.measurement_list)
                meas = np.array( [d['power'] for d in self.measurement_list])
                idx = np.where(np.logical_and(meas>=SIG_BOUND_LOW, meas<=SIG_BOUND))[0]
                meas = np.array( [d['power'] for i, d in enumerate(self.measurement_list) if i in idx ])
                lats = np.array( [d['lat'] for i, d in enumerate(self.measurement_list) if i in idx ] )
                lons = np.array( [d['lon'] for i, d in enumerate(self.measurement_list) if i in idx ] )

                #print("Was going E/W ", self.heading_seq_w_idx)

                dist = 385.58835496039694 + -7.879822789610755*meas  
                # estimated latitudes of rover
                est_lon = [ geopy.distance.distance(meters = d).destination(point=geopy.Point(la, lo), bearing=270).longitude for la, lo, d in zip(lats, lons, dist) ]
                if len(est_lon):
                    #print("Updating longitude from %f to %f based on %d measurements from %f to %f" % (self.best_pos.lon, np.median(est_lon), len(est_lon), np.min(meas), np.max(meas) ) ) 
                    self.best_pos.lon = np.median(est_lon)
                    self.oob_best_pos['lon'] = np.median(est_lon)
                print("Position estimate: ", self.best_pos.lat, self.best_pos.lon, datetime.datetime.now() - self.start_time)

            # reset measurement list
            self.measurement_list = []

            self.heading_seq_w_idx = (self.heading_seq_w_idx+1) % 8

            if (self.heading_seq_w_idx > len(HEADING_SEQ_W) -1 ):
                self.heading_seq_w_idx = 0
                self.probe_state = STATE_STEADY
                print("Setting next state to STEADY")

                return "suggest"

            return "suggest_seq"

    @state(name="suggest")
    async def suggest(self, vehicle: Drone):
        #print("Suggesting next waypoint using the optimizer")
        # suggest the next waypoint to visit
        self.next_waypoint = self.optimizer.suggest(self.utility)
        # go to next waypoint
        return "probe"
    
    @state(name="suggest_seq")
    async def suggest_seq(self, vehicle: Drone):
        #print("Suggesting next waypoint by sequence")

        # suggest the next waypoint to visit
        # go back to "best" position 
        if (
            ( (self.probe_state == STATE_OOB_N) and (HEADING_SEQ_N[self.heading_seq_n_idx] == -1) ) or
            ( (self.probe_state == STATE_OOB_W) and (HEADING_SEQ_W[self.heading_seq_w_idx] == -1) ) 
        ):
            #print("Suggest_seq: next point should be best known position")
            self.next_waypoint = {'lat': self.best_pos.lat, 'lon': self.best_pos.lon}

        # go up to 100m away from "best" position 
        elif (self.probe_state == STATE_OOB_N):
            #print("Suggest: for %d, next point should be 100m toward bearing %d" % ( self.heading_seq_n_idx, HEADING_SEQ_N[self.heading_seq_n_idx]) )
            start_point = geopy.Point(vehicle.position.lat, vehicle.position.lon)
            next_point = geopy.distance.distance(meters = 100).destination(point=start_point, bearing=HEADING_SEQ_N[self.heading_seq_n_idx]) 
            self.next_waypoint = {'lat': next_point.latitude, 'lon': next_point.longitude}
        elif (self.probe_state == STATE_OOB_W):
            #print("Suggest: for %d, next point should be 100m toward bearing %d" % ( self.heading_seq_w_idx, HEADING_SEQ_W[self.heading_seq_w_idx]) )
            start_point = geopy.Point(vehicle.position.lat, vehicle.position.lon)
            next_point = geopy.distance.distance(meters = 100).destination(point=start_point, bearing=HEADING_SEQ_W[self.heading_seq_w_idx]) 
            self.next_waypoint = {'lat': next_point.latitude, 'lon': next_point.longitude}

        # go to next waypoint
        return "probe"

    

    @state(name="probe")
    async def probe(self, vehicle: Drone):

        # stop if search time is over
        if datetime.datetime.now() - self.start_time > self.search_time:
            return "end"

        # go to the waypoint, probe along the way
        next_pos =  Coordinate(
            np.clip(self.next_waypoint['lat'], MIN_LAT, MAX_LAT),
            np.clip(self.next_waypoint['lon'], MIN_LON, MAX_LON),
            SEARCH_ALTITUDE)
        #(valid_waypoint, msg) = self.safety_checker.validateWaypointCommand(
        #    vehicle.position, next_pos
        #)
        valid_waypoint = True
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

                pos = vehicle.position
                self.measurement_list.append( 
                    {'lat': pos.lat, 'lon': pos.lon, 'power': measurement} 
                    )

                await asyncio.sleep(0.2)

            #print("Now at: ", pos.lat, pos.lon, measurement )

                
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

        #xvalues = np.linspace(MIN_LAT, MAX_LAT, num=500)
        #yvalues = np.linspace(MIN_LON, MAX_LON, num=500)
        #a,b = np.meshgrid(xvalues,yvalues)
        #positions = np.vstack([a.ravel(), b.ravel()])
        #x_test = (np.array(positions)).T
        #y_test = self.optimizer._gp.predict(x_test)
        #idx_max = np.argmax(y_test)
        #print(
        #    f"Max of GP estimate {x_test[idx_max,0], x_test[idx_max,1]} with predicted measurement {y_test[idx_max]}"
        #)
        home_coords = Coordinate(
            vehicle.home_coords.lat, vehicle.home_coords.lon, vehicle.position.alt
        )



        #with open('gaussian_process.pickle', 'wb') as handle:
        #    pickle.dump(self.optimizer._gp, handle)

        await vehicle.goto_coordinates(home_coords)
        await vehicle.land()
        print("Done!")
