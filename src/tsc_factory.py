from src.trafficsignalcontrollers.uniformcycletsc import UniformCycleTSC
from src.trafficsignalcontrollers.websterstsc import WebstersTSC
from src.trafficsignalcontrollers.pretimedtsc import PreTimedTSC
from src.trafficsignalcontrollers.maxpressuretsc import MaxPressureTSC
from src.trafficsignalcontrollers.sotltsc import SOTLTSC
# from src.trafficsignalcontrollers.nextphaserltsc import NextPhaseRLTSC
# from src.trafficsignalcontrollers.nextdurationrltsc import NextDurationRLTSC
# from src.rl_factory import rl_factory

def tsc_factory(tsc_type, tsc_id, args, netdata, conn, tsc_data=None):
    '''
    For non-rl tsc only, simplify the inputs
    '''

    if tsc_type == 'websters':
        return WebstersTSC(conn, tsc_id, args.mode, netdata, args.r, args.y,
                           args.g_min, args.c_min,
                           args.c_max, args.sat_flow,
                           args.update_freq, args.save_tsc)
    elif tsc_type =='pretimed':
        return PreTimedTSC(conn, tsc_id, args.mode, netdata, args.r, args.y, tsc_data, args.sim_len, args.save_tsc) 

    elif tsc_type == 'uniform':
        return UniformCycleTSC(conn, tsc_id, args.mode, netdata, args.r, args.y, args.g_min)


def _tsc_factory(tsc_type, tl, args, netdata, rl_stats, exp_replay, neural_network, eps, conn):
    if tsc_type == 'websters':
        return WebstersTSC(conn, tl, args.mode, netdata, args.r, args.y,
                           args.g_min, args.c_min,
                           args.c_max, args.sat_flow,
                           args.update_freq)
    elif tsc_type =='pretimed':
        return PreTimedTSC(conn, tl, args.mode, netdata, args.r, args.y, args.tscfp) 
    
    elif tsc_type == 'sotl':
        return SOTLTSC(conn, tl, args.mode, netdata, args.r, args.y,
                       args.g_min, args.theta, args.omega,
                       args.mu )
    elif tsc_type == 'uniform':
        return UniformCycleTSC(conn, tl, args.mode, netdata, args.r, args.y, args.g_min)
    elif tsc_type == 'maxpressure':
        return MaxPressureTSC(conn, tl, args.mode, netdata, args.r, args.y,
                              args.g_min )
    elif tsc_type == 'dqn':
        dqnagent = rl_factory(tsc_type, args,
                              neural_network, exp_replay, rl_stats, len(netdata['inter'][tl]['green_phases']), eps)
        return NextPhaseRLTSC(conn, tl, args.mode, netdata, args.r, args.y,
                              args.g_min, dqnagent)
    elif tsc_type == 'ddpg':
        ddpgagent = rl_factory(tsc_type, args,
                                neural_network, exp_replay, rl_stats, 1, eps)
        return NextDurationRLTSC(conn, tl, args.mode, netdata, args.r, args.y,
                                 args.g_min, args.g_max, ddpgagent)
    else:
        #raise not found exceptions
        assert 0, 'Supplied traffic signal control argument type '+str(tsc)+' does not exist.'
