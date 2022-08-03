import os, sys

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
    from sumolib import checkBinary
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import traci

import os, sys
import numpy as np
import traci

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
    from sumolib import checkBinary
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")


class TrafficMonitor:
    '''
    Monitoring traffic states by road link (edge)
    '''
    def __init__(self, link_id, conn, netdata, camInfo=None, whole_link=True):
        # print('the monitor id is {}'.format(idx))
        self.id = link_id # the edge id
        self.length = netdata['edge'][self.id]['length'] # link_length
        self.camInfo = camInfo
        if whole_link: # if observe the traffic state along the whole link
            self.start_point, self.end_point = self.get_camera_range()
            dist = self.length
        else:
            self.start_point, self.end_point = self.get_camera_range() # start, end points of the traffic monitor
            dist = self.length - self.start_point
        
        self.conn = conn
        self.conn.edge.subscribeContext(self.id, traci.constants.CMD_GET_VEHICLE_VARIABLE, dist, 
                [traci.constants.VAR_LANE_ID,
                traci.constants.VAR_LANEPOSITION])
    
    def run(self):
        """Implement this function to perform any traffic monitor updates, e.g., traffic camera, edge monitor, etc. 
        """
        vehData = self.get_subscription_data()
        if self.camInfo:
            return (len(vehData), self.get_camera_data(vehData))
        else:
            return len(vehData)
        

    def get_subscription_data(self):
        '''
        use SUMO subscription to retrieve vehicle information:
        https://sumo.dlr.de/docs/TraCI/Interfacing_TraCI_from_Python.html
        Get metadata for update:
        1. number of vehicles (by lane) --> len(vehicle id list)
        2. average vehicle travel time (by lane) # TODO later
        3. simulated camera data (len(vehicle id list))
        '''
        vehData = self.conn.edge.getContextSubscriptionResults(self.id)
        # {'veh_id': {
        #     traci.constants.VAR_LANE_ID: --,
        #     traci.constants.VAR_LANEPOSITION: --}}

        # print('the veh data is {}'.format(veh_data.keys()))
        return vehData

    def get_camera_data(self, vehData):
        '''use SUMO subscription to retrieve vehicle information:
        https://sumo.dlr.de/docs/TraCI/Interfacing_TraCI_from_Python.html
        Get metadata for update:
        1. number of vehicles (by lane) --> len(vehicle id list)
        2. average vehicle travel time (by lane) # TODO later
        3. simulated camera data (len(vehicle id list))
        '''
        cam_veh = set()

        for v in vehData:
            pos = vehData[v][traci.constants.VAR_LANEPOSITION] # vehicle position is recorded as location from the start-point of a directed road link
            if pos >= self.start_point and pos < self.end_point: 
                cam_veh.add(v)
            
        # print('the edge is {}'.format(self.id))
        # print('the num veh is {}'.format(num_veh)) 
        # print('the num of detected veh is {}'.format(len(cam_veh)))                     
        return len(cam_veh)

    def get_camera_range(self):
        '''
        get start and end positions of the traffic monitor
        '''
        if self.camInfo == None:
            return (self.length, -1)
            
        start, length = self.camInfo['start'], self.camInfo['length']
        end_pos = (1 - start) * self.length
        
        if length < self.length:
            start_pos = end_pos - length
        else:
            start_pos = 0

        return (start_pos, end_pos)

        
class CamSelector:
    def __init__(self, all_edges, penetration=1):
        self.edges = all_edges
        self.p = penetration

    def selectByRandom(self):
        '''
        select by using np.random.choice
        '''
        if self.p >= 1:
            return self.edges
        
        np.random.seed(40)
        select_size = int(self.p * len(self.edges))
        
        return np.random.choice(self.edges, size=select_size, replace=False)

    def select_all(self):
        return self.edges



class TrafficMetrics:
    def __init__(self, _id, incoming_lanes, netdata, metric_args, mode):
        self.metrics = {}
        if 'delay' in metric_args:
            lane_lengths = {lane:netdata['lane'][lane]['length'] for lane in incoming_lanes}
            lane_speeds = {lane:netdata['lane'][lane]['speed'] for lane in incoming_lanes}
            self.metrics['delay'] = DelayMetric(_id, incoming_lanes, mode, lane_lengths, lane_speeds )

        if 'queue' in metric_args:
            self.metrics['queue'] = QueueMetric(_id, incoming_lanes, mode)

    def update(self, v_data):
        for m in self.metrics:
            self.metrics[m].update(v_data)

    def get_metric(self, metric):
        return self.metrics[metric].get_metric()

    def get_history(self, metric):
        return self.metrics[metric].get_history()

class TrafficMetric:
    def __init__(self, _id, incoming_lanes, mode):
        self.id = _id
        self.incoming_lanes = incoming_lanes
        self.history = []
        self.mode = mode

    def get_metric(self):
        pass

    def update(self):
        pass

    def get_history(self):
        return self.history

class DelayMetric(TrafficMetric):
    def __init__(self, _id, incoming_lanes, mode, lane_lengths, lane_speeds):
        super().__init__( _id, incoming_lanes, mode)
        self.lane_travel_times = {lane:lane_lengths[lane]/float(lane_speeds[lane]) for lane in incoming_lanes}
        self.old_v = set()
        self.v_info = {}
        self.t = 0

    def get_v_delay(self, v):
        return ( self.t - self.v_info[v]['t'] ) - self.lane_travel_times[self.v_info[v]['lane']]

    def get_metric(self):
        #calculate delay of vehicles on incoming lanes
        delay = 0
        for v in self.old_v:
            #calculate individual vehicle delay
            v_delay = self.get_v_delay(v)
            if v_delay > 0:
                delay += v_delay

        return delay

    def update(self, v_data):
        new_v = set()

        #record start time and lane of new_vehicles
        for lane in self.incoming_lanes:
            for v in v_data[lane]:
                if v not in self.old_v:
                    self.v_info[v] = {}
                    self.v_info[v]['t'] = self.t
                    self.v_info[v]['lane'] = lane
            new_v.update( set(v_data[lane].keys()) )

        if self.mode == 'test':
            self.history.append(self.get_metric())

        #remove vehicles that have left incoming lanes
        remove_vehicles = self.old_v - new_v
        delay = 0
        for v in remove_vehicles:
            del self.v_info[v]
        
        self.old_v = new_v
        self.t += 1

class QueueMetric(TrafficMetric):
    def __init__(self, _id, incoming_lanes, mode):
        super().__init__( _id, incoming_lanes, mode)
        self.stop_speed = 0.3
        self.lane_queues = {lane:0 for lane in self.incoming_lanes}

    def get_metric(self):
        return sum([self.lane_queues[lane] for lane in self.lane_queues])

    def update(self, v_data):
        lane_queues = {}
        for lane in self.incoming_lanes:
            lane_queues[lane] = 0
            for v in v_data[lane]:
                if v_data[lane][v][traci.constants.VAR_SPEED] < self.stop_speed:
                    lane_queues[lane] += 1

        self.lane_queues = lane_queues
        if self.mode == 'test':
            self.history.append(self.get_metric())

