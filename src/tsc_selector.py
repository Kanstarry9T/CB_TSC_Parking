'''
Design traffic signal controller selection algorithms
'''

import numpy as np

class Selector:
    def __init__(self, all_tscs, penetration):
        self.tscs = all_tscs
        self.p = penetration

    def selectByRandom(self):
        '''
        select by using np.random.choice
        '''
        if self.p >= 1:
            return self.tscs
        
        np.random.seed(40)
        select_size = int(self.p * len(self.tscs))
        
        return np.random.choice(self.tscs, size=select_size, replace=False)

    def selectByVolume(self):
        '''
        select by total volume of all incoming lanes
        '''
        pass

    def selectByCrit(self):
        '''
        select by link criticality
        '''

    def selectByQuality(self):
        '''
        select by link service quality (average speed / max speed)
        '''