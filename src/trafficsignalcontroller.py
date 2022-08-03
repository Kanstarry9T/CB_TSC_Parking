import os, sys

from collections import deque
from itertools import cycle

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
    from sumolib import checkBinary
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import traci

from src.trafficmonitor import TrafficMetrics

'''
Implement TSC with NEMA ring-barrier definition:
http://www.dot.state.mn.us/trafficeng/publ/signaloperations/2013_Signal_Opt_and_Timing_Manual.pdf
https://sumo.dlr.de/docs/Simulation/Traffic_Lights.html#loading_new_tls-programs
'''
''' 
    References:
        https://ops.fhwa.dot.gov/publications/fhwahop08024/chapter4.htm
    
    NEMA phase definition: clockwise labeling, start from northmost incoming edge, for a standard 4-leg intersection:
        - 1, 3, 5, 7 for left-turn & u-turn of intersection leg 0, 1, 2, 3
        - 2, 4, 6, 8 for straight-turn of intersection leg 0, 1, 2, 3
        - by default, right-turn is with minor green (g)
    
    Data structure
        'split': [[(), ()], [(), ()]]
        'flow': [[(), ()], [(), ()]]
        'duration': [[(), ()], [(), ()]]
        'start': [[(), ()], [(), ()]]


    Examples
        Example-0: "lead-lead" sequence which has both opposing left-turn phases ending at the same time
            1  6 | 3  8 |
            -----|------|
            5  2 | 7  4 | 

        Example-1: "lag-lag" sequence which has both opposing left-turn phases ending at the same time
            6  1 | 8  3 |
            -----|------|
            2  5 | 4  7 | 
        
        Example-2: "lead-lag" left-turn, indicating that same approach left-turn & straight starting at the same time
            6  1 | 8  3 |
            -----|------|
            5  2 | 4  7 | 

        Example-3: for 3-leg intersection
            1  |  6     |  
            ---|--------|
               |  5  4  |
'''
    

class TrafficSignalController:
    """Abstract base class for all traffic signal controller.

    Build your own traffic signal controller by implementing the follow methods.
    """
    def __init__(self, conn, tsc_id, mode, netdata, g_min, red_t, yellow_t, save_signal=None):

        # traffic signal and network data
        self.id = tsc_id
        self.netdata = netdata
        self.inter_data = self.netdata['inter'][self.id]
        self.tls_lane = self.inter_data['tlsindex'] #tlsindex: lane_id 
        self.lane_data = self.netdata['lane'] # lane_id: edge, outgoing, incoming, movement, speed, length
        self.incoming_lanes = set(self.lane_data.keys())

        # default yellow and red time for phase switch 
        self.red_t = red_t
        self.yellow_t = yellow_t
        self.g_min = g_min

        self.nyrs = 4 # todo next: determine number of yellow_red switches based on netdata
        self.leadLag_mode = 'lag_lag'
        
        # for pretimed tsc, save the historical signal timing
        self.save_signal = save_signal
        self.signal_timing = dict()

       
        # run simulation and get subscribed traffic data 
        self.conn = conn
        dist = 50 # Upstream from the intersection for vehicle data collection
        self.conn.junction.subscribeContext(self.id, traci.constants.CMD_GET_VEHICLE_VARIABLE, dist, 
            [traci.constants.VAR_LANEPOSITION, 
            traci.constants.VAR_SPEED, 
            traci.constants.VAR_LANE_ID]) # create subscription to get vehicle data 
        #Please refer to https://sumo.dlr.de/docs/TraCI/Object_Context_Subscription.html
        
    '''
    Step 1: initialize NEMA phases
        step 1.1: get NEMA phases information 
        step 1.2: split NEMA phasing using ring-barrier diagram & initialize NEMA phase instances with the corresponding (lanes, tlindices data, default duration) (https://ops.fhwa.dot.gov/publications/fhwahop08024/chapter4.htm)
        step 1.3: convert NEMA phases to standard phases & get standard phase cycle data
        step 1.4: set initial SUMO phase states
    '''

    '''
    Step 2: run simulation & update NEMA phase duration and tsc timing incrementally
    ''' 
    def run(self):
        data = self.get_subscription_data()
        # self.trafficmetrics.update(data)
        self.update(data)
        self.increment_controller()
    
    def get_subscription_data(self):
        #use SUMO subscription to retrieve vehicle info in batches
        # https://sumo.dlr.de/docs/TraCI/Interfacing_TraCI_from_Python.html
        #around the traffic signal controller
        tl_data = self.conn.junction.getContextSubscriptionResults(self.id)                          
        #create empty incoming lanes for use else where
        lane_vehicles = {l:{} for l in self.incoming_lanes}
        if tl_data is not None:
            for v in tl_data:
                lane = tl_data[v][traci.constants.VAR_LANE_ID]
                if lane not in lane_vehicles:
                    lane_vehicles[lane] = {}
                lane_vehicles[lane][v] = tl_data[v] 
        # print('the lane_vehicles is {}'.format(lane_vehicles))
        return lane_vehicles

    def update(self, data):
        """Implement this function to perform any
           traffic signal class specific control/updates 
        """
        raise NotImplementedError("Subclasses should implement this!")

    def increment_controller(self):
        raise NotImplementedError("Subclasses should implement this!")

    def get_signal(self):
        '''
        return recorded signal timing data
        '''
        return self.signal_timing
