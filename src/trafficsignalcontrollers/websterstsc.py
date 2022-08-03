from audioop import lin2adpcm
from itertools import cycle
from collections import deque
from copy import deepcopy
import numpy as np
import math

from src.trafficsignalcontroller import TrafficSignalController

'''
Implement webster with NEMA ring-barrier definition:
http://www.dot.state.mn.us/trafficeng/publ/signaloperations/2013_Signal_Opt_and_Timing_Manual.pdf
https://sumo.dlr.de/docs/Simulation/Traffic_Lights.html#loading_new_tls-programs
'''
'''

    Phase split examples
        Example-0: "lead-lead" sequence which has both opposing left-turn phases starting at the same time, odd number for left-turn, even number for straight
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
'''
lane_vehicle data --> update NEMA phase data --> convert to standard phase --> set next cycle phase timing
'''

'''
TODO: develop a NEMA Phase labeling algorithm for TSC controller
'''

class NEMAPhase:
    '''
    Define NEMA phases
    '''
    def __init__(self, NEMA_id, tlidx, lanes, duration=None):
        self.id = NEMA_id
        self.lanes = lanes
        self.tlidx = tlidx
        self.duration = duration
        self.init_flow()
        
    def init_flow(self):
        '''
        initialize flows by lane
        '''
        self.flow = {lane: 0 for lane in self.lanes}
    
    def update_flow(self, lane):
        '''
        update NEMA Phase volume (or flow) by adding 1 vehicle to the lane 
        '''
        self.flow[lane] += 1
    
    def get_critFlow(self):
        '''
        return the critical flow of the NEMA phase
        '''
        return max(self.flow.values())
    
    def set_duration(self, duration):
        self.duration = duration

class WebstersTSC(TrafficSignalController):
    '''
    Define Websters Traffic signal controller
    '''
    def __init__(self, conn, tsc_id, mode, netdata, red_t, yellow_t, g_min, c_min, c_max, sat_flow=0.38, update_freq=1, save_signal=False):
        super().__init__(conn, tsc_id, mode, netdata, g_min, red_t, yellow_t, save_signal)
        
        # default parameters for signal timing
        self.c_min = c_min
        self.c_max = c_max 
        self.update_freq = update_freq # default update_freq = 1 (unit: cycle)
        self.sat_flow = sat_flow
        self.min_diff = (self.yellow_t + self.red_t) * 2

        # run simulation and record phase cycles
        self.t = 0
        self.phase_time = 0
        self.phase_lanes = None
        self.nps = None # current pairwise NEMA Phases
        self.nPhases = list()
        self.phaseCycle = None 
        self.durationCycle = None 
        self.npsCycle = None # cycle of pairwise NEMA phases 
        self.lanesCycle = None # cycle of controlled lanes
        self.prevCycle_data = None


        # update data and signal timing
        self.prev_data = None
        self.phase_switch = 0
        self.cycle_cnt = 0
        self.cnt_duration = 0 # time duration for counting lane vehicle flow

        # record the signal timing data
        self.cycleLengths = list()
        self.npSplits = list() # NEMA phase split durations
        self.satRates = list() # record the saturation flow rates

        # initialize signal timing using NEMA phase definition
        self.initialize()

    def initialize(self):
        '''
        intialize NEMA phases data
        '''
        
        self.get_NEMAInfo() # step 1.1
        # if self.id == 'A0':
        #     print(self.NEMAInfo)
        #     print(self.inter_data)
        self.init_NEMAphase(self.leadLag_mode, self.nyrs) # step 1.2 initialize NEMA phases
        self.get_phaseCycle() # # step 1.3 get standard phase cycles
    
    def update(self, data):
        #step-1: update vehicle counts by NEMA phase

        self.set_NEMAphaseFlow(data)

        if self.phase_switch == self.nPhases[-1]:
            self.cycle_cnt += 1
            self.phase_switch = 0
            if self.cycle_cnt % self.update_freq == 0: # update the signal timing if update frequency is met
                self.cnt_duration = sum(self.cycleLengths[-self.update_freq:])
                self.set_NEMAPhaseDuration() # run websters' method to update NEMA phase durations
                self.get_phaseCycle() # update phase cycle using updated NEMA phase durations
                self.init_NEMAPhaseFlow() # set NEMA phase volume to zero to record vehicle counts in the coming signal cycle 
            else:
                self.get_phaseCycle(previous=True) # use signal timing from previous cycle
        
        self.prev_data = data
        self.t += 1
    
    def increment_controller(self):
        '''
        traffic signal controller switch signal phase
        '''
        if self.phase_time == 0:
            ###get new phase and duration
            next_phase = self._next_phase()
            self.phase_lanes = self._next_phaseLanes()
            self.nps = self._next_NEMAphases()
            self.conn.trafficlight.setRedYellowGreenState(self.id, next_phase)
            self.phase_switch += 1
            # print('phase switch is {}'.format(self.phase_switch))

            # print('the current phase is {}'.format(self.phase))
            self.phase_time = self._next_phaseDuration()
            # print('the time is {}, the phase is {}'.format(self.t, next_phase))
            
            if self.save_signal:
                signal_dict = {self.t-1: next_phase}
                self.signal_timing.update(signal_dict)

        self.phase_time -= 1

    def get_NEMAInfo(self):
        tlsindexdir = self.inter_data['tlsindexdir'] # tlsindex: turn
  
        u_turn = sorted([k for k, v in tlsindexdir.items() if v == 't'])
        left = sorted([k for k, v in tlsindexdir.items() if v == 'l'])
        straight = sorted([k for k, v in tlsindexdir.items() if v == 's'])
        # print(straight)
        
        self.rightTlidx = sorted([k for k, v in tlsindexdir.items() if v == 'r'])
        
        # update left and u-turn NEMA data

        if len(u_turn) > 0:
            # print('get u_turn NEMA data for inter-{}'.format(self.id))
            uturn_dict = dict() # u-turn edge dictionary data
            laneTlidx= [(self.tls_lane[tlidx], tlidx) for tlidx in u_turn]
            # laneEdges = [('-gneE8_2', '-gneE8', 20)] # for test only

            for lane, tl_idx in laneTlidx:
                edge = self.lane_data[lane]['edge']
                if edge in uturn_dict:
                    uturn_dict[edge].append((tl_idx, lane))
                else:
                    uturn_dict.update({edge: [(tl_idx, lane)]})
            # print('uturn_dict is {}'.format(uturn_dict))
        else:
            uturn_dict = None
        
        # if self.id == 'A0':
        #     print(uturn_dict)
        nema_data = self._get_NEMALaneTlidx(left, id_type='odd', uturn_dict=uturn_dict)

        # for straight turn
        nema_data0 = self._get_NEMALaneTlidx(straight, id_type='even')
        nema_data.update(nema_data0)

        self.NEMAInfo = nema_data
        
    def init_NEMAphase(self, leadLag_mode, nyrs):
        if nyrs == 4:
            default_splits = {
                'lead_lead': [[1, 6], [3, 8], [5, 2], [7, 4]],
                'lag_lag': [[6, 1], [8, 3], [2, 5], [4, 7]],
                'lead_lag': [[6, 1], [8, 3], [5, 2], [4, 7]],
            }
        else:
            raise NotImplementedError("ring_barrier diagrams for {}-yellow-red switches intersection will be implemented later!".format(nyrs))
        
        splits = default_splits[leadLag_mode]
        duration = self.g_min
        NEMAPhases = [
            (NEMAPhase(id0, self.NEMAInfo[id0]['tlidx'], self.NEMAInfo[id0]['lanes'], duration), 
            NEMAPhase(id1, self.NEMAInfo[id1]['tlidx'], self.NEMAInfo[id1]['lanes'], duration)) 
            for id0, id1 in splits]
        
        self.barrier1 = NEMAPhases[0] + NEMAPhases[2]
        self.barrier2 = NEMAPhases[1] + NEMAPhases[3]
        self.NEMAPhases = NEMAPhases
        
    def get_phaseCycle(self, previous=False):
        '''
        Convert NEMA barrier data to the standard "phase-cycle" data structure, more friendly for setting SUMO signal tming
        '''
        if previous: # use the signal timing data of previous signal cycle
            self.phaseCycle, self.durationCycle, self.lanesCycle, self.npsCycle = self.prevCycle_data
            self.nPhases.append(self.nPhases[-1])
            self.cycleLengths.append(self.cycleLengths[-1])
            self.npSplits.append(self.npSplits[-1])
            
            # print('nema_phase splits are {}'.format(self.npSplits))
            return

        phase_states1, durations1, phase_NEMA1, phase_lanes1 = self._get_barrierData(self.barrier1)
        phase_states2, durations2, phase_NEMA2, phase_lanes2 = self._get_barrierData(self.barrier2)

        phase_cycle = phase_states1 + phase_states2 # phase state cycle - SUMO phase state
        duration_cycle = durations1 + durations2 # phase durations
        lanes_cycle = phase_lanes1 + phase_lanes2
        nps_cycle = phase_NEMA1 + phase_NEMA2
        
        self.nPhases.append(len(phase_cycle))
        # print('number of phases is {}'.format(self.nPhases))
        # print('the phase cycle is {}'.format(phase_cycle))

        self.phaseCycle = cycle(phase_cycle)
        self.durationCycle = cycle(duration_cycle)
        self.lanesCycle = cycle(lanes_cycle)
        self.npsCycle = cycle(nps_cycle)
        self.cycleLengths.append(sum(duration_cycle))

        self.prevCycle_data = (self.phaseCycle, self.durationCycle, self.lanesCycle, self.npsCycle)
        # print('cycle length is {}'.format(self.cycleLengths))

        tmp = dict()
        [tmp.update({np0.id: np0.duration, np1.id: np1.duration}) for np0, np1 in self.NEMAPhases]
        self.npSplits.append(tmp)

    def _get_NEMALaneTlidx(self, turn_tlidx, id_type='even', uturn_dict=None):
        '''
        whether a tlidx belongs to the same edge, same type of turn
        '''
        if len(turn_tlidx) <= 0:
            print("There is no tlidx for intersection {} with this turn movement {}!".format(self.id, id_type))
            return None
        
        if id_type == 'even':
            idx = 2
        else:
            idx = 1
        # whether the next idx belongs to the same edge?
        
        tlidx = turn_tlidx.pop(0)
        prev_lane = self.tls_lane[tlidx]
        prev_edge = self.lane_data[prev_lane]['edge']

        nema_data = {idx: {'tlidx': [tlidx], 'lanes': [prev_lane]}}

        # update u-turn data if necessary
        if uturn_dict and prev_edge in uturn_dict:
            # if self.id == 'A0':
            #     print(prev_edge, uturn_dict[prev_edge])
            [(nema_data[idx]['tlidx'].append(d[0]), nema_data[idx]['lanes'].append(d[1]) if d[1] not in nema_data[idx]['lanes'] else None) for d in uturn_dict[prev_edge]]

        while len(turn_tlidx) > 0:
            tlidx = turn_tlidx.pop(0)
            lane = self.tls_lane[tlidx]
            edge = self.lane_data[lane]['edge']


            if edge == prev_edge:
                nema_data[idx]['tlidx'].append(tlidx)
                nema_data[idx]['lanes'].append(lane)
                
            else:
                idx += 2
                nema_data[idx] = {'tlidx': [tlidx], 'lanes': [lane]} 
                # update u-turn tlidx and lanes
                if uturn_dict and edge in uturn_dict:
                    # if self.id == 'A0':
                    #     print(edge, prev_edge, uturn_dict[edge])
                    [(nema_data[idx]['tlidx'].append(d[0]), nema_data[idx]['lanes'].append(d[1])) for d in uturn_dict[edge]]
            prev_edge = edge

        return nema_data

    def _get_barrierData(self, barrier):
        '''
        get standard phases within a barrier
        '''
        # step-1 parse barrier 0, 2 to get remain phase & duration
        phase_NEMA = list()
        durations = list()
        np0, np1, np2, np3 = barrier
        remain_dur = np0.duration - np2.duration
        yr_dur = self.yellow_t + self.red_t # assuming remain duration > yr_dur, see the predefined min difference (min_diff) in non-conflicting pairwise NEMAphases durations

        yr = [self.yellow_t, self.red_t] if self.red_t > 0 else [self.yellow_t]
        k = 3 if self.red_t > 0 else 2


        if remain_dur > 0:
            nps = [(np0, np2), (np0, np3), (np1, np3)]
            phase_NEMA = [(np0, np2)] * k + [(np0, np3)] *k + [(np1, np3)] * k
            durations = [np2.duration] + yr + [remain_dur-yr_dur] + yr + [np1.duration] + yr# need to consider the transit phase (i.e., yellow & red)
            phase_lanes = [(np0.lanes, np2.lanes)] * k + [(np0.lanes, np3.lanes)] * k + [(np1.lanes, np3.lanes)] * k


        elif remain_dur == 0:
            nps = [(np0, np2), (np1, np3)]
            phase_NEMA = [(np0, np2)] * k + [(np1, np3)] * k
            durations = [np0.duration] + yr + [np1.duration] + yr
            phase_lanes = [(np0.lanes, np2.lanes)] * k + [(np1.lanes, np3.lanes)] * k 
    
        else:
            nps = [(np0, np2), (np1, np2), (np1, np3)]
            phase_NEMA = [(np0, np2)] * k + [(np1, np2)] * k + [(np1, np3)] * k
            durations = [np0.duration] + yr + [- remain_dur-yr_dur] + yr + [np3.duration] + yr
            phase_lanes = [(np0.lanes, np2.lanes)] * k + [(np1.lanes, np2.lanes)] * k + [(np1.lanes , np3.lanes)] * k         
        # print('length of phase nema is {}'.format(len(phase_NEMA)))
        # phase_list, durations = update_redYellow(phase_list, durations)
        phase_states = self._get_phaseStates(nps)
        return phase_states, durations, phase_NEMA, phase_lanes
    
    def _get_phaseStates(self, phases):
        res_states = list()
        states = list()

        for p in phases:
            state = self._convert_phase2State(p)
            states.append(state)

        next_states = states[1:] + [states[0]]
    
        for s, next_s in zip(states, next_states):
            # print(s)
            transit_s = self._get_transitPhases(s, next_s)
            res_states.append(s)
            res_states += transit_s
        # print(res_states)
        return res_states
    
    def _convert_phase2State(self, phase):
        total_tlidx = list(self.tls_lane.keys())
        right_turns = self.rightTlidx

        state = [0] * len(total_tlidx)
        
        tlidx_G = phase[0].tlidx + phase[1].tlidx if len(phase) > 1 else phase[0].tlidx
        
        for idx in total_tlidx:
            if idx in tlidx_G:
                state[idx] = 'G'

            elif idx in right_turns:
                state[idx] = 'g'
            else:
                state[idx] = 'r'
        
        return ''.join(state)

    def _get_transitPhases(self, state, next_state):
        '''
        get transition yellow & red phase states between current & next phases 
        '''
        y_trans = ''.join(['y' if s1 == 'G' and s2 == 'r' else s1 for s1, s2 in zip(state, next_state) ])

        result = [y_trans]
        if self.red_t > 0:
            r_trans = ''.join(['r' if s1 == 'G' and s2 == 'r' else s1 for s1, s2 in zip(state, next_state) ])
            result  += [r_trans]

        # print(result)

        return result

    def set_NEMAphaseFlow(self, data):
        '''
        :update observed flow of NEMA phase critical lanes within {} signal cycle(s)
        '''
        # count the new-out vehicles, that is, the vehicle in prev_data but not in data

        # if lane in prev_lanes and cur_lanes

        if self.prev_data:
            # step - 1: find vehicle set in current data
            if len(self.phase_lanes) > 1: # case 1: standard phasing for pairwise straight & left-turn movements
                cur_veh1, cur_veh2 = set([]), set([]) 
                l1, l2 = self.phase_lanes
                

                np1, np2 = self.nps

                for ln1 in l1:
                    for v1 in data[ln1]:
                        cur_veh1.add(v1)
                
                for ln2 in l2:
                    for v2 in data[ln2]: #lane change before crossing the intersection is considered
                        cur_veh2.add(v2)
            
                # step - 2: filter-out those vehicle just exited the lane, i.e., the vehicles in prev_data but not in (current) data

                for ln in l1:
                    for v1 in self.prev_data[ln]:
                        if v1 not in cur_veh1: # if not in cur_vehicles ~= those vehicles exited the lane
                            np1.update_flow(ln)
                for ln in l2:   
                    for v2 in self.prev_data[ln]:
                        if v2 not in cur_veh2: # if not in cur_vehicles ~= those vehicles exited the lane
                            np2.update_flow(ln)
            else: # case 2: special phasing for single movement
                cur_veh = set([])                
                for v in [data[l] for l in self.phase_lanes]:
                    cur_veh.add(v)
                
                for ln in self.phase_lanes:
                    for v1 in self.prev_data[ln]:
                        if v1 not in cur_veh: # if not in cur_vehicles ~= those vehicles exited the lane
                            self.nps.update_flow(ln)

    def init_NEMAPhaseFlow(self):
        # if there is a NEMA phase switch, set all lane flows to 0
        [nema_p.init_flow() for nema_p in np.array(self.NEMAPhases).flatten()]

    def set_NEMAPhaseDuration(self, method='websters'):
        '''
        compute NEMA phase durations using websters' method
        '''
        ##compute flow ratios for all lanes in all green phases
        ##find critical 

        y_b1, volumes_b1 = self._get_barrierFlowRatio(self.barrier1)
        y_b2, volumes_b2 = self._get_barrierFlowRatio(self.barrier2)
        #compute intersection critical lane flow rattios
        Y = y_b1 + y_b2
        total_vol = Y * (self.sat_flow * 3600)
        # print('the total flow ratio is {}'.format(Y))
        # print('the total flow is {} per hour'.format(total_vol))
        
        if Y > 0.95:
            Y = 0.95

        self.satRates.append(Y)

        #limit in case too saturated
        #compute lost time
        L = self.nyrs * (self.red_t + self.yellow_t)

        #compute cycle time
        if method == 'websters':
            C = int(((1.5*L) + 5)/(1.0-Y))
        else: # use default desired cycle length method
            PHF = 0.9 # Peak hour factor: .85-.95 to account peak hour 15 minutes traffic
            v_c = .9 # target v/c ratio for the critcal movements in the intersection
            C = L / (1 - (total_vol / (3600 * self.sat_flow * PHF * v_c)))
        #constrain if necessary
        if C > self.c_max:
            C = self.c_max
        elif C < self.c_min:
            C = self.c_min

        if Y == 0:
            C = self.c_min
            G1, G2 = .5*C, .5*C
        else:
            G = C - L
        #compute green times for each movement
        #based on total green times
            G1 = G * y_b1 / max(Y, y_b1 + y_b2) 
            G2 = G * y_b2 / max(Y, y_b1 + y_b2) 
        
        self._set_barrierDurations(self.barrier1, volumes_b1, G1)
        self._set_barrierDurations(self.barrier2, volumes_b2, G2)

    def _get_barrierFlowRatio(self, barrier):
        '''
        compute flow ratio using critical lane flow data of each NEMA phase
        '''
        # step - 1: get the total critical flow ratio of the barrier
        crit_volumes = [npi.get_critFlow() for npi in barrier]
        crit_volume1 = crit_volumes[0] + crit_volumes[1]
        crit_volume2 = crit_volumes[2] + crit_volumes[3]
        # print('in {}, the barrier critcal volume are {}, {}, saturate flow is {}'.format(self.cnt_duration, crit_volume1, crit_volume2, self.sat_flow * self.cnt_duration))
        crit_flowRatio = max(crit_volume1, crit_volume2) / (self.sat_flow * self.cnt_duration)

        return crit_flowRatio, crit_volumes
    
    def _set_barrierDurations(self, barrier, volumes, total_duration):
        '''
        set NEMA phases durations by barrier
        total_duration: total effective green 
        '''
        volumes_r1 = volumes[:2] # ring1 volumes
        volumes_r2 = volumes[2:] # ring2 volumes

        total_vol1 = sum(volumes_r1)
        total_vol2 = sum(volumes_r2)

        if total_vol1 > 0:
            duration_r1 = [max(self.g_min, math.ceil(total_duration*v/total_vol1)) for v in volumes_r1]
        else:
            duration_r1 = [self.g_min for _ in volumes_r1]

        if total_vol2 > 0:
            duration_r2 = [max(self.g_min, math.ceil(total_duration*v/total_vol2)) for v in volumes_r2]
        else:
            duration_r2 = [self.g_min for _ in volumes_r2]

        # to do next: if abs(d1-d2) <= self.min_diff, d1=d2= max(d1,d2)
        if abs(duration_r1[0] - duration_r2[0]) <= self.min_diff:
            duration_r1[0], duration_r2[0] = [math.ceil(max(duration_r1[0], duration_r1[0]))] * 2 
        
        if abs(duration_r1[1] - duration_r2[1]) <= self.min_diff:
            duration_r1[1], duration_r2[1] = [math.ceil(max(duration_r1[1], duration_r1[1]))] * 2

        durations = duration_r1 + duration_r2
        
        [barrier[i].set_duration(durations[i]) for i in range(len(barrier))]
    
    '''
    helper functions: update signal phases
    '''
    def _next_phase(self):
        return next(self.phaseCycle)

    def _next_phaseDuration(self):
        return next(self.durationCycle)

    def _next_phaseLanes(self):
        return next(self.lanesCycle)
    
    def _next_NEMAphases(self):
        return next(self.npsCycle)