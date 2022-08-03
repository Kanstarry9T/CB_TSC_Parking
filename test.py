from itertools import cycle
import numpy as np
import json
from src.trafficsignalcontrollers.websterstsc import WebstersTSC, NEMAPhase
import os
import pandas as pd

scenario =  'hangzhou'
net_json = 'netdata.json'
out_folder = 'output/' + scenario + '0'
netfp = os.path.join(out_folder, net_json)


with open(netfp, 'r') as f:
    netdata = json.load(f)  

def analyze_inter():
    interdata = netdata['inter']
    inter_incomings = {k: len(interdata[k]['incoming']) for k in interdata.keys()}
    inter_outgoings = {k: len(interdata[k]['outgoing']) for k in interdata.keys()}

    data = list()
    cols = ['inter_id', 'incomings', 'outgoings']

    for k in interdata.keys():
        row = [k, inter_incomings[k], inter_outgoings[k]]
        data.append(row)
    
    df = pd.DataFrame(data, columns=cols)
    csv_fp = os.path.join(out_folder, 'interdata.csv')
    df.to_csv(csv_fp)

def get_inter(inter_id):
    interdata = netdata['inter']
    inter_id = str(inter_id)
    print('The data for {} is {}'.format(inter_id, interdata[inter_id]))

if __name__ == '__main__':
    # analyze_inter()
    get_inter(2375)