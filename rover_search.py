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


BOUND_NE={'lon':-78.69621514941473, 'lat':35.72931030026633}
BOUND_NW={'lon':-78.69953825817279, 'lat':35.72931030026633}
BOUND_SE={'lon':-78.69621514941473, 'lat':35.72688213193035}
BOUND_SW={'lon':-78.69953825817279, 'lat':35.72688213193035}


MAX_LON = BOUND_NE['lon']
MIN_LON = BOUND_NW['lon']
MAX_LAT = BOUND_NE['lat']
MIN_LAT = BOUND_SE['lat']

SEARCH_ALTITUDE = 50 # in meters


WAYPOINTS = [
    Coordinate(BOUND_SE['lat'], BOUND_SE['lon'], SEARCH_ALTITUDE)
]


difference_lat=(BOUND_NE['lat'] - BOUND_SE['lat'])/10
difference_lon=(BOUND_NE['lon'] - BOUND_NW['lon'])/10
for i in range(5):
    current_lon=BOUND_NW['lon']+difference_lon*i
    current_lat=BOUND_NW['lat']-difference_lat*i
    WAYPOINTS.append(Coordinate(current_lat,current_lon,SEARCH_ALTITUDE))

    current_lon=BOUND_NE['lon']-difference_lon*i
    current_lat=BOUND_NE['lat']-difference_lat*i
    WAYPOINTS.append(Coordinate(current_lat,current_lon,SEARCH_ALTITUDE))

    current_lon=BOUND_SE['lon']-difference_lon*i
    current_lat=BOUND_SE['lat']+difference_lat*i
    WAYPOINTS.append(Coordinate(current_lat,current_lon,SEARCH_ALTITUDE))

    current_lon=BOUND_SW['lon']+difference_lon*i
    current_lat=BOUND_SW['lat']+difference_lat*i
    WAYPOINTS.append(Coordinate(current_lat,current_lon,SEARCH_ALTITUDE))




class RoverSearch(StateMachine):
    last_measurement = float("-inf")
    best_measurement = float("-inf")
    best_pos: Coordinate = None
    start_time = None
    search_time = None

    waypoint_idx = 0

    measurement_list = []
    # start at the SE bound
    next_waypoint = WAYPOINTS[waypoint_idx]

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

        return "probe"


    @state(name="register")
    async def register(self, vehicle: Drone):
            
        if self.save_csv:
            for m in self.measurement_list:
                self.csv_writer.writerow(
                    [datetime.datetime.now() - self.start_time, m['lat'], m['lon'], SEARCH_ALTITUDE, m['power']]
                )

        # reset measurement list
        self.measurement_list = []

        return "probe"    

    @state(name="probe")
    async def probe(self, vehicle: Drone):

        # stop if we've visted the last waypoint
        if self.waypoint_idx >= len(WAYPOINTS):
            return "end"

        # go to the waypoint, probe along the way
        next_pos =  WAYPOINTS[self.waypoint_idx]

        valid_waypoint = True
        if valid_waypoint:
            moving = asyncio.ensure_future(
                vehicle.goto_coordinates(next_pos)
            )
            while not moving.done():
                # Open data buffer
                f = open("/root/Power", "rb")
                # unpack binary reading into a float
                measurement_from_file = unpack("<f", f.read(4))
                measurement = measurement_from_file[0]
                # close the data buffer
                f.close()

                pos = vehicle.position
                self.measurement_list.append( 
                    {'lat': pos.lat, 'lon': pos.lon, 'power': measurement} 
                    )

                await asyncio.sleep(0.2)

        # increment the waypoint index
        self.waypoint_idx = self.waypoint_idx + 1

        return "register"


    @state(name="end")
    async def end(self, vehicle: Drone):
        # Return vehicle to start and land it

        home_coords = Coordinate(
            vehicle.home_coords.lat, vehicle.home_coords.lon, vehicle.position.alt
        )
        await vehicle.goto_coordinates(home_coords)
        await vehicle.land()
        print("Done!")
