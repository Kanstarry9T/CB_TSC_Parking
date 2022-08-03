from ast import Assert
import os, sys, subprocess

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
    from sumolib import checkBinary
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import traci
import numpy as np
import json
from copy import deepcopy

from src.tsc_factory import tsc_factory
# from src.tsc_selector import Selector

from src.parking import ParkingMonitor


class SumoSim:
    def __init__(self, cfg_fp, sim_len, tsc, gui, netdata, args, idx, pids=["PA_B1B2"]):
        self.cfg_fp = cfg_fp
        self.sim_len = sim_len
        self.tsc_type = tsc
        self.tscs = None
        # print('the gui is set to {}'.format(gui))
        self.sumo_cmd = 'sumo-gui' if gui else 'sumo'
        self.netdata = netdata
        self.args = args
        self.idx = idx
        self.pids = pids # parking ID list

    def gen_sim(self):
        #create sim stuff and intersections
        #serverless_connect()
        #self.conn, self.sumo_process = self.server_connect()

        port = self.args.port+self.idx
        sumoBinary = checkBinary(self.sumo_cmd)
        port = self.args.port+self.idx
        self.sumo_process = subprocess.Popen([sumoBinary, "-c",
                                         self.cfg_fp, "--remote-port",
                                         str(port), "--no-warnings",
                                         "--no-step-log", "--random"],
                                         stdout=None, stderr=None)

        self.conn = traci.connect(port)

        self.t = 0
        self.v_start_times = dict()
        self.vehTT = dict() # vehicle travel time data
        self.vehTD = dict() # vehicle travel distance data
        self.vehSet = set([]) # set of all vehicles that are currently in this scenario 
        self.departedNumber = list()
        self.arrivedNumber = list()
        self.abnormal_veh = list()

        # self.vehiclegen = None
        # if self.args.sim == 'double' or self.args.sim == 'single':
        #     self.vehiclegen = VehicleGen(self.netdata, 
        #                                  self.args.sim_len, 
        #                                  self.args.demand, 
        #                                  self.args.scale,
        #                                  self.args.mode, self.conn) 
    
    def run(self):
        #execute simulation for desired simulation time length
        # print('THE SIMULATION STEP LENGTH IS {}'.format(self.conn.simulation.getDeltaT()))
        # create traffic signal controllers
        self.create_tscs()
        self.create_parkingMonitors(self.pids)

        # run simulations
        while self.t < self.sim_len:

            self.update_vehData()
            #run all traffic signal controllers in network
            for tsc_id in self.tscs:
                self.tscs[tsc_id].run()

            #run parking monitor & update parking data
            for pid in self.pms:
                self.pms[pid].run()
            
            self.sim_step()

        #post-process to save signal timing data
        # print('In exp {}, the abnormal vehicle are {}!'.format(self.idx, self.abnormal_veh))
        if self.args.save_tsc:
            self.save_tscs()
    
    def create_parkingMonitors(self, pids):
        self.pms = {pid: ParkingMonitor(pid, self.conn) for pid in pids}
            
    def save_tscs(self):
        '''
        save all tsc timings
        '''
        save_path = self.args.save_tsc
        signal_data = {tsc_id: self.tscs[tsc_id].get_signal() for tsc_id in  list(self.tscs.keys())}

        # dump signal timing data into a json file
        with open(save_path, 'w') as f:
            json.dump(signal_data, f)

        
    def create_tscs(self, default_tsc='websters'):
        
        if self.tsc_type == 'pretimed':
            load_path = self.args.load_tsc
            
            with open(load_path, 'r') as f:
                tsc_dict = json.load(f)

            all_tls = list(tsc_dict.keys())
            self.tl_junc = set(all_tls)
            

            penetrated_tls = self._select_tsc(self.args.p, all_tls)
            pretimed_tls = self.tl_junc - set(penetrated_tls)
            print("For exp-{}, the penetration rate is {}".format(self.idx, self.args.p))
            print("The penetrated {} tscs are {}".format(len(penetrated_tls), penetrated_tls))

            self.tscs = {tsc_id: tsc_factory(self.args.tsc, tsc_id, self.args, self.netdata, self.conn, tsc_dict[tsc_id]) for tsc_id in pretimed_tls}

            penetrated_tsc = { tsc_id:tsc_factory(default_tsc, tsc_id, self.args, self.netdata, self.conn) for tsc_id in penetrated_tls }
            self.tscs.update(penetrated_tsc)

            # print("There are {} tscs in total".format(len(self.tsc)))
        
        else:
            self.tl_junc = self.get_traffic_lights() 
            self.tscs = { tsc_id:tsc_factory(self.args.tsc, tsc_id, self.args, self.netdata, self.conn) for tsc_id in self.tl_junc }

    def _select_tsc(self, p, all_tls):
        if p >= 1:
            return all_tls
        
        np.random.seed(40)
        select_size = int(p * len(all_tls))
        return np.random.choice(all_tls, size=select_size, replace=False)


    def serverless_connect(self):
        traci.start([self.sumo_cmd, 
                     "-c", self.cfg_fp, 
                     "--no-step-log", 
                     "--no-warnings",
                     "--random"])

    def server_connect(self):
        sumoBinary = checkBinary(self.sumo_cmd)
        port = self.args.port+self.idx
        sumo_process = subprocess.Popen([sumoBinary, "-c",
                                         self.cfg_fp, "--remote-port",      
                                         str(port), "--no-warnings",
                                         "--no-step-log", "--random"],
                                         stdout=None, stderr=None)

        return traci.connect(port), sumo_process
    
    def get_traffic_lights(self):
        #find all the junctions with traffic lights
        trafficlights = self.conn.trafficlight.getIDList()
        junctions = self.conn.junction.getIDList()

        tl_juncs = set(trafficlights).intersection( set(junctions) )
        tls = list()
     
        # double-check traffic light junctions, only keep traffic lights with more than 1 green phase
        for tl in tl_juncs:
            #subscription to get traffic light phases
            self.conn.trafficlight.subscribe(tl, [traci.constants.TL_COMPLETE_DEFINITION_RYG])
            tldata = self.conn.trafficlight.getAllSubscriptionResults()
            logic = tldata[tl][traci.constants.TL_COMPLETE_DEFINITION_RYG][0]

            green_phases = [ p.state for p in logic.getPhases()
                             if 'y' not in p.state
                             and ('G' in p.state or 'g' in p.state) ]
            if len(green_phases) > 1:
                tls.append(tl)

        return set(tls) 

    def update_netdata(self):
        tl_junc = self.get_traffic_lights()
        all_intersections = set(self.netdata['inter'].keys())
        #only keep intersections that we want to control
        for i in all_intersections - tl_junc:
            # print('the intersection - {} will be deleted!'.format(i))
            del self.netdata['inter'][i]

        return self.netdata

    def sim_step(self):
        self.conn.simulationStep()
        self.t += 1

    def run_offset(self, offset):
        while self.t < offset:
            #create vehicles if vehiclegen class exists
            if self.vehiclegen:
                self.vehiclegen.run()
            self.update_travel_times()
            self.sim_step()

    def update_vehData(self):
        '''
        Update vehicle data step-by-step, refer to following links for data retrieval:
            https://sumo.dlr.de/docs/TraCI/Simulation_Value_Retrieval.html
            https://sumo.dlr.de/docs/TraCI/Vehicle_Value_Retrieval.html
        '''
        departed_veh = self.conn.simulation.getDepartedIDList()
        arrived_veh = self.conn.simulation.getArrivedIDList()
        self.departedNumber.append(len(departed_veh))
        self.arrivedNumber.append(len(arrived_veh))

        '''process departed vehicles'''
        self.vehSet = self.vehSet.union(departed_veh)    
        for v in departed_veh:
            self.v_start_times[v] = self.t

        '''process arrived vehicles'''
        # self._update_vehStats(arrived_veh)  # if vehicle arrived, make a last updating on its travel time and travel distance
        self.vehSet -= set(arrived_veh) # delete the the arrived vehicles, only keep the on-road vehicle
        '''process on-road vehicles '''
        self._update_vehStats(self.vehSet)

    def _update_vehStats(self, vehSet):
        for v in vehSet:
            travel_time = self.t - self.v_start_times[v]
            travel_dist = self.conn.vehicle.getDistance(v)
    
            if travel_dist < 0:
                self.abnormal_veh.append(v)
                continue # do not update the abnormal vehicle 
            else:
                self.vehTT[v] = travel_time
                self.vehTD[v] = travel_dist

    def sim_stats(self):
        # get total travel time and travel distance
        totalN = len(self.vehTT)
        totalTT = sum(self.vehTT.values())
        totalTD = sum(self.vehTD.values())

        res_total = {'totalN': totalN, 'totalTT': totalTT,
                    'avgTT': totalTT/totalN, 'totalTD': totalTD,
                    'avgTD': totalTD/totalN, 'avgPace': totalTT/totalTD, 
                    'avgSpeed': totalTD/totalTT, 'throughput': self.departedNumber[-1]}
        res_detail = {'arrivedNumber': self.arrivedNumber, 'departedNumber': self.departedNumber, 'travel_time': self.vehTT, 'travel_distance': self.vehTD}

        return res_total, res_detail
        


    def get_tsc_metrics(self):
        tsc_metrics = {}
        for tsc in self.tsc:
            tsc_metrics[tsc] = self.tsc[tsc].get_traffic_metrics_history()
        return tsc_metrics

    def close(self):
        #self.conn.close()
        self.conn.close()
        self.sumo_process.terminate()
