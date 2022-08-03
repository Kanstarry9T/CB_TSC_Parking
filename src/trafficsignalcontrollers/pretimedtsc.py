from itertools import cycle
from collections import deque

from src.trafficsignalcontroller import TrafficSignalController

'''
https://sumo.dlr.de/docs/Simulation/Traffic_Lights.html#loading_new_tls-programs
'''


class PreTimedTSC(TrafficSignalController):
    def __init__(self, conn, tsc_id, mode, netdata, red_t, yellow_t, tsc_data, sim_len, save_signal):
        super().__init__(conn, tsc_id, mode, netdata, red_t, yellow_t, save_signal)

        self.t = 0
        self.phase_time = 0
        self.sim_len = sim_len
        self.tscdata = tsc_data
        self.prev_data = None
        self.switch_t = [int(time) for time in self.tscdata.keys()] # The system times for phase switching
        self.switch_t.sort() 

    def update(self, data):
        # self.prev_data = data
        self.t += 1

    def increment_controller(self):
        '''
        traffic signal controller switch signal phase
        '''
        if self.phase_time == 0:
            ###get new phase and duration
            next_phase = self._next_phase()
            self.conn.trafficlight.setRedYellowGreenState(self.id, next_phase)
            
            # print('phase switch is {}'.format(self.phase_switch))

            # print('the current phase is {}'.format(self.phase))
            self.phase_time = self._next_phaseDuration()
            # print('the time is {}, the phase is {}'.format(self.t, next_phase))
            
            if self.save_signal:
                signal_dict = {self.t-1: next_phase}
                self.signal_timing.update(signal_dict)

        self.phase_time -= 1

    def _next_phase(self):
        return self.tscdata[str(self.t-1)]

    def _next_phaseDuration(self):
        
        phase_start = int(self.switch_t.pop(0))
        if len(self.switch_t) > 0:
            phase_end = self.switch_t[0]
            phase_duration = phase_end - phase_start
        else:
            phase_duration = self.sim_len - self.t + 1

        return phase_duration

