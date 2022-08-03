import numpy as np
import matplotlib.pyplot as plt
import os
import json

scenario = 'grid'
out_folder = 'output/' + scenario + '1'
param = 'p'
tsc = 'pretimed'
data = [.1 * ii for ii in range(11)]

def get_results():
    x = list()
    y_speed = list()
    totalN = list()
    for d in data:
        fp = os.path.join(out_folder, tsc + '_'+param+str(int(10*d)) +'_res.json')
        with open(fp, 'r') as f:
            dd = json.load(f)
            x.append(int(100*d))
            # y_throughput.append(dd['throughput'])
            y_speed.append(dd['avgSpeed'])
            totalN.append(dd['totalN'])
            if d<= 0:
                y_baseline = dd['avgSpeed']
    N = int(np.mean(totalN))
    return x, y_baseline, y_speed, N

def plot_line(x, y, ylabel, color='b'):
    print(x, y)
    plt.plot(x, y, color=color)
    plt.title(ylabel)
    plt.xlim([0, 100])
    # plt.ylim([min(y) - 1000, max(y) + 1000])
    plt.ylabel(ylabel)
    plt.xlabel('Penetration of smart TSC (%)')
    plt.savefig(os.path.join(out_folder, ylabel+'.svg'))
    # plt.tight_layout()
    # plt.show()
    plt.close()

if __name__ == '__main__':
    x, y_baseline, y_speed, N = get_results()
    plot_line(x, y_speed, 'Average speed')
    # y_min = min(y_speed)
    y_percent = [100 * (y - y_baseline)/y for y in y_speed]
    plot_line(x, y_percent, 'Improvement (%)', color='g')

    # quantify time_savings per km
    y_pace = [1/s for s in y_speed]
    y_pace_diff = [1000/3600 * N * (1/y_baseline-y) for y in y_pace]
    plot_line(x, y_pace_diff, 'total time savings among' + str(N) + 'vehicles hours per km', color='g')
    y_pace_diff = [1000 * (1/y_baseline-y) for y in y_pace]
    plot_line(x, y_pace_diff, 'total time savings per vehicle seconds per km', color='b')

