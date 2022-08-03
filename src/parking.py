'''
Helper class for parkings:
    - Parking monitor, please refer to https://sumo.dlr.de/docs/TraCI/Simulation_Value_Retrieval.html#generic_parameter_retrieval_0x7e  
    - Control vehicles' parking behavior using vehicle value retrieval https://sumo.dlr.de/docs/TraCI/Vehicle_Value_Retrieval.html

'''
import os, sys
from tracemalloc import stop
import numpy as np
import traci

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
    from sumolib import checkBinary
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")


class ParkingMonitor:
    '''
    Monitoring real-time parking status
    - absolute available parking spaces
    - parking occupancy
    '''
    def __init__(self, pid, conn):
        self.id = pid
        self.conn = conn

        # parking vehicleID subscriptions
        # self.conn.simulation.subscribe(varIDs=(
        #     traci.VAR_PARKING_STARTING_VEHICLES_IDS,
        #     traci.VAR_PARKING_ENDING_VEHICLES_IDS
        # ))

    def run(self):
        self.cap, self.occ = self.get_parkingData()
        # print('the parking lot {} with cap-{}, occ-{}'.format(self.id, self.cap, self.occ))

    def get_parkingData(self):
        cap = int(self.conn.simulation.getParameter(self.id, 'parkingArea.capacity'))
        occ = int(self.conn.simulation.getParameter(self.id, 'parkingArea.occupancy'))

        return cap, occ


class ParkingVehicle:
    '''
    Parking vehicle state getter: https://sumo.dlr.de/docs/TraCI/Vehicle_Value_Retrieval.html
    Parking vehicle state setter: https://sumo.dlr.de/docs/TraCI/Change_Vehicle_State.html

    Parking vehicles rerouting logics: find the nearest parkingArea with largest probability of availibility
    '''
    def __init__(self, vid, conn, args):
        '''
        step-1: Get vehicular next stops, either a parkingArea or destination
        step-2: if it's a parkingArea, get the number of empty spaces travel-time, and further select the parkingAreas
        step-3: If necessary， reroute the vehicle to a selected parkingArea , use vehicle.rerouteParkingArea or vehicle.setRoute 
        step-4: get the route to vehicular final destination
        '''
        self.id = vid # vehicle id
        self.args = args # IDs of candidate parkingAreas 
        self.conn = conn # sumo connection port
        self.stopType = None
        self.pid = None # parkingArea ID
    
    def run(self):
        self.getNextStop()
        self.checkParkingState()
        self.setRoute()
    
    def get_plannedTraj(self):
        '''
        get the initially planned trajectory
        '''

    def get_parkingRRs(self):
        '''
        return a dictionary that describe the 
        '''
    def getNextStop(self):
        '''
        return
            1) next stop type
            2) next stop information
        '''
        nextStop = self.conn.vehicle.getNextStops(self.id)
        self.stopType = 'parking'
        return nextStop
    
    def checkParkingState(self):
        parkingSpaces = dict()
        for pid in self.pids:
            pm = ParkingMonitor(pid, self.conn)
            cap, occ = pm.get_parkingData()
            parkingSpaces[pid] = cap - occ

    def setRoute(self):
        if self.stopType == 'parking':
            print('The vehicle {} reroute from parking {} to {}'.format(self.id, self.pid))
            self.conn.vehicle.rerouteParkingArea(self.id, self.pid)
        else:
            edges, _ = self._findRoute(self.pid, self.dest)
            self.conn.vehicle.setRoute(self.id, edges)

    def _findRoute(self, origin, dest):
        route = self.conn.simulation.findRoute(origin, dest)
        edges = route.edges
        travelTime = route.travelTime

        return travelTime, edges        


