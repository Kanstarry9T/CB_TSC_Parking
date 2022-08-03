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
from src.argparse import parse_cl_args
from src.helper_funcs import create_folder
from src.networkdata import NetworkData
from src.sumosim import SumoSim
from src.distprocs import DistProcs

import subprocess
from multiprocessing import Barrier
from src.simproc import SimProc


from scenarios import randomTrips

scenario =  'double'
sim_len = 3600

# run multiple parallelized processes with different penetration rate - p

def init_run():
    start_t = time.time()
    print('start initial run...')

    work_folder = 'scenarios/' + scenario + '0'
    out_folder = 'output/' + scenario + '0'

    create_folder(work_folder) 
    create_folder(out_folder)
  

    # step 1: generate trips
    gen_randomTrips(work_folder)
    
    # step 2: run initial simulation to get recorded signal timings
    tsc = 'websters'
    save_tsc = os.path.join(out_folder, tsc+'.json')


    # netdata = run(work_folder, out_folder, tsc, save_tsc)

    params = [(work_folder, out_folder, tsc, save_tsc, None, 0)]
    netdata = run(params, gui=False)

    print('initial running time '+str((time.time()-start_t)/60))

    return work_folder, save_tsc, netdata

def second_run(src_folder, src_tsc, netdata):
    start_t = time.time()
    print('start second run...')

    assert src_tsc, 'You need enter the path for loading for pretimed tsc'
    work_folder = 'scenarios/' + scenario + '1'
    out_folder = 'output/' + scenario + '1'

    create_folder(work_folder) 
    create_folder(out_folder)

    # step 0 copy net and cfg files
    files = ['net.net.xml', 'cfg.sumocfg', 'vtype.xml']
    [_copy_file(os.path.join(src_folder, f), work_folder) for f in files]

    # step 1: generate trips
    gen_randomTrips(work_folder, seed=30, p=.2)

    # step 2: run comparative sumo simulations
    # step 2.1 run pretimed traffic signal
    # step 2.2 run websters traffic signal
    

    tsc = 'pretimed'
    penetrations = [.1 * ii for ii in range(11)]
    penetrations = [0, 0.5]
    params = [
        (work_folder, out_folder, tsc, os.path.join(out_folder, tsc+'_p'+str(int(10*p))+'.json'), src_tsc, p) for p in penetrations
    ]
    # params.append((work_folder, out_folder, 'websters', save_tsc, None, 0))

    
    run(params, netdata, False)

    print('second running time (in minutes) '+str((time.time()-start_t)/60))

def _copy_file(src_file, target_folder):
    cmd = ['cp', src_file, target_folder]  
    process = subprocess.Popen(cmd)
    process.wait()


def _get_args(params, gui):
    args_list = list()
    for param in params:

        args = parse_cl_args()

        args.sim_len = sim_len
        args.gui = gui
        args.sim = scenario
        work_folder, out_folder, args.tsc, args.save_tsc, args.load_tsc, args.p = param
        args.net_fp = os.path.join(work_folder, 'net.net.xml')
        args.cfg_fp = os.path.join(work_folder, 'cfg.sumocfg')  
        args.csv_path = os.path.join(out_folder, str(args.sim)+str(args.tsc) +'_p'+str(int(10*args.p))+'.csv')
        

        args_list.append(args)

    return args_list


def run(params, netdata=None, gui=False):
    #step-1 get netdata by creating a dummy sim 
    args_list = _get_args(params, gui)

    if not netdata:
        print('step 1: create dummy sim for netdata...')

        args = args_list[0]
        nd = NetworkData(args.net_fp)
        netdata = nd.get_net_data()
        sim = SumoSim(args.cfg_fp, args.sim_len, args.tsc, args.gui, netdata, args, -1)
        sim.gen_sim()
        netdata = sim.update_netdata()
        sim.close()
        tsc_ids = netdata['inter'].keys()
        print("Step 1 finished, the {} signalized intersections are {}".format(len(tsc_ids), tsc_ids))


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


def gen_randomTrips(folder, seed=42, net='net.net.xml', trip='trips.xml', add_file='vtype.xml', p=.2):
    cmd = ['python', '../randomTrips.py', '-n', net, '-o', trip, '-a', add_file, '-p', str(p), '-s', str(seed), '-e', str(sim_len),'--validate']

    process = subprocess.Popen(cmd, cwd=folder)
    process.wait()
    print("Finished random trip generation")



if __name__ == '__main__':
    # folder='scenarios/grid0'
    # gen_randomTrips(folder)
    # prev_folder='scenarios/' + scenario + '0'
    # prev_tsc = 'output/' + scenario + '0/' + 'tsc.json'
    prev_folder, prev_tsc, netdata = init_run()
    second_run(prev_folder, prev_tsc, netdata)
