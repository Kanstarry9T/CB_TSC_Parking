'''
Pipeline for run experiment - 1
:step-1: initial run to get selected traffic signal timings
    step-1.1 generate initial trips
    step-1.2 run initial simulation & save initial traffic signal timings
:step-2: second run to compare pretimed vs. adaptive (websters) traffic signal control
    step-2.1 re-generate trips (with minor variations on initial trips) to simulate day-to-day variation on traffic flow
    step-2.2 run second simulation & compare trip delays between pretimed vs. smart adaptive tsc

'''
import os, sys, time
import json

from numpy import save
from src.argparse import parse_cl_args
from src.helper_funcs import create_folder
from src.networkdata import NetworkData
from src.sumosim import SumoSim
# from copy import deepcopy

import subprocess
from multiprocessing import Barrier
from src.simproc import SimProc
# import argparse

# from scenarios import randomTrips

scenario =  'grid'
sim_len = 3600
gui = True
net_json = 'netdata.json'
out_folder = 'output/' + scenario + '0'
netfp = os.path.join(out_folder, net_json)


def optimize_hp():
    '''
    optimize hyper-parameters for TSC: e.g., sat_flow, cmax, cmin, gmin
    '''
    param = 'satFlow'
    data = [0.3, 0.38, 0.44, .5, .6]
    data = [0.5]

    print("start to optimize hyper-parmater {} at scenario {}".format(param, scenario))

    start_t = time.time()
    print('start initial run...')

    src_folder = 'scenarios/' + scenario + '0'
    work_folder = 'scenarios/' + scenario + '_hp'
    out_folder = 'output/' + scenario + '_hp'

    create_folder(work_folder) 
    create_folder(out_folder)

    files = ['net.net.xml', 'cfg.sumocfg', 'vtype.xml']
    [_copy_file(os.path.join(src_folder, f), work_folder) for f in files]
  

    # step 1: generate trips
    gen_randomTrips(work_folder, p=.05)
    
    # step 2: run initial simulation to get recorded signal timings
    tsc = 'websters'
    save_tsc = os.path.join(out_folder, tsc+'.json')
    save_tsc = None


    # netdata = run(work_folder, out_folder, tsc, save_tsc)


    args_list = update_args(work_folder, out_folder, tsc, data, param)
    netdata = run(args_list)

    print('initial running time '+str((time.time()-start_t)/60))

def init_run():
    '''
    initial run to generate benchmark pretimed  TSC timings based on traffic flow0
    '''
    start_t = time.time()
    print('start initial run...')

    work_folder = 'scenarios/' + scenario + '0'
    out_folder = 'output/' + scenario + '0'

    create_folder(work_folder) 
    create_folder(out_folder)
  
    # step 1: generate trips
    # gen_randomTrips(work_folder, p=.1)
    
    # step 2: run initial simulation to get recorded signal timings
    tsc = 'websters'
    save_tsc = os.path.join(out_folder, tsc+'_timing.json')
    args_list = update_args(work_folder, out_folder, tsc, saveLoad=(save_tsc, None), update_freq=5)
    netdata = run(args_list)


    # netdata = run(work_folder, out_folder, tsc, save_tsc)

    print('initial running time '+str((time.time()-start_t)/60))

    return work_folder, save_tsc, netdata

def second_run(src_folder, load_tsc, netdata):
    '''
    Second run for experiments
    '''
    start_t = time.time()
    print('start second run...')

    assert load_tsc, 'You need enter the path for loading for pretimed tsc'
    work_folder = 'scenarios/' + scenario + '1'
    out_folder = 'output/' + scenario + '1'

    create_folder(work_folder) 
    create_folder(out_folder)

    # step 0 copy net and cfg files
    files = ['net.net.xml', 'cfg.sumocfg', 'vtype.xml']
    [_copy_file(os.path.join(src_folder, f), work_folder) for f in files]

    # step 1: generate trips
    gen_randomTrips(work_folder, seed=30, p=.1)

    # step 2: run comparative sumo simulations
    # step 2.1 run pretimed traffic signal
    # step 2.2 run websters traffic signal
    

    tsc = 'pretimed'
    data = [.1 * ii for ii in range(11)]
    # penetrations = [0, 0.8]
    args_list = update_args(work_folder, out_folder, tsc, saveLoad=(None, load_tsc), data=data, param='p')
    run(args_list, netdata)

    print('second running time (in minutes) '+str((time.time()-start_t)/60))

def update_args(work_folder, out_folder, tsc, saveLoad=(None, None), data=None, param=None, update_freq=None):
    
    args_list = list()
    if data:
        for d in data:
            
            args = parse_cl_args()
            args.sim_len = sim_len
            args.gui = gui
            args.sim = scenario
            args.net_fp = os.path.join(work_folder, 'net.net.xml')
            args.cfg_fp = os.path.join(work_folder, 'cfg.sumocfg')  
            
            args.tsc = tsc
            args.save_tsc = saveLoad[0]
            args.load_tsc = saveLoad[1]
            args.res_fp = os.path.join(out_folder, tsc + '_'+param+str(int(10*d)) +'_res.json')
            if update_freq:
                args.update_freq = update_freq
            

            if param == 'p':
                args.p = d
            
            if param == 'cmin':
                args.c_min = d

            if param == 'cmax':
                args.c_max = d

            if param == 'satFlow':
                args.sat_flow = d
            
            if param == 'gmin':
                args.g_min = d
            
            print(args.res_fp)
            args_list.append(args)
    else:
        args = parse_cl_args()
        args.sim_len = sim_len
        args.gui = gui
        args.sim = scenario
        args.net_fp = os.path.join(work_folder, 'net.net.xml')
        args.cfg_fp = os.path.join(work_folder, 'cfg.sumocfg')  
        
        args.tsc = tsc
        args.save_tsc = saveLoad[0]
        args.load_tsc = saveLoad[1]
        args.res_fp = os.path.join(out_folder, tsc + '_init_res.json')
        args_list.append(args)

    return args_list

def run(args_list, netdata=None):
    #step-1 get netdata by creating a dummy sim 

    if not netdata:
        print('step 1: create dummy sim for netdata...')

        args = args_list[0]
        netdata = get_netdata(args)


    # step-2 run sumo simulation & write results to csv
    print('step 2: run sumo simulation...')


    barrier = Barrier(len(args_list))
    sim_procs = [ SimProc(i, args_list[i], barrier, netdata) for i in range(len(args_list))]

    print('Starting up all processes...')
        ###start everything   
    for p in sim_procs:
        p.start()
                              
        ###join when finished
    for p in sim_procs:
        p.join()

    print('...finishing all processes')
 
    return netdata

def gen_randomTrips(folder, seed=42, net='net.net.xml', trip='trips.xml', add_file='vtype.xml', p=.15):
    '''
    generate background with randomTrips.py
    '''
    cmd = ['python', '../randomTrips.py', '-n', net, '-o', trip, '-a', add_file, '-p', str(p), '-s', str(seed), '-e', str(sim_len),'--validate']

    process = subprocess.Popen(cmd, cwd=folder)
    process.wait()
    print("Finished random trip generation")

def get_netdata(args):
    
    nd = NetworkData(args.net_fp)
    netdata = nd.get_net_data()
    sim = SumoSim(args.cfg_fp, args.sim_len, args.tsc, args.gui, netdata, args, -1)
    sim.gen_sim()
    netdata = sim.update_netdata()
    sim.close()
    tsc_ids = netdata['inter'].keys()

    with open(netfp, 'w') as f:
        json.dump(netdata, f)

    print("Step 1 finished, the {} signalized intersections are {}".format(len(tsc_ids), tsc_ids))
    return netdata

def _copy_file(src_file, target_folder):
    cmd = ['cp', src_file, target_folder]  
    process = subprocess.Popen(cmd)
    process.wait()
    print("Finished copy {} to {}".format(src_file, target_folder))

if __name__ == '__main__':
    # folder='scenarios/grid0'
    # gen_randomTrips(folder)
    # prev_folder='scenarios/' + scenario + '0'
    # prev_tsc = 'output/' + scenario + '0/' + 'tsc.json'
    # optimize_hp()
    # NEMA_test()
    # work_folder = os.path.join('scenarios', 'grid0')
    # cmd = ['python', '../parkingGenerator.py']
    # process = subprocess.Popen(cmd, cwd=work_folder)
    # process.wait()
    # cmd1 = ['python', '../tripGenerator.py']
    # process1 = subprocess.Popen(cmd1, cwd=work_folder)
    # process1.wait()
    prev_folder, prev_tsc, netdata = init_run()
    # second_run(prev_folder, prev_tsc, netdata)
