'''
Generate parking data
 - Parking area (and rerouter)
 - Parking trips generation?
'''
'''
Reference:
- https://sumo.dlr.de/docs/Simulation/ParkingArea.html
- https://sumo.dlr.de/docs/Simulation/Rerouter.html#rerouting_to_an_alternative_parking_area
'''

'''
xml file operation: https://stackabuse.com/reading-and-writing-xml-files-in-python/

'''

import os, sys

# solving the path problem

import numpy as np
from networkdata import NetworkData
import xml.dom.minidom as minidom
from scipy.spatial.transform import Rotation as R

scenario =  'grid'
simLen = 3600
# net_json = 'netdata.json'
# out_folder = 'output/' + scenario + '0'
scen_folder = './' + scenario + '0'
net_fp = os.path.join(scen_folder, 'net.net.xml')

# parking additional-files
parking_add = 'parkings.add.xml'
parkingRR_add = 'parkingRerouters.add.xml'

parking_fp = os.path.join(scen_folder, parking_add)
parkingRR_fp = os.path.join(scen_folder, parkingRR_add)

nd = NetworkData(net_fp)
netdata = nd.get_net_data()
linkdata = netdata['edge'] # {'link_id': 'lanes': }
demo_link = list(linkdata.keys())[0]
print('the demo link is {}'.format(linkdata[demo_link]))

def process_links(linkdata):
    '''
    filter out those sublinks (roadway channelization at intersections)
    '''
    sublink_map = dict() # map sublinks' id to links' id
    res = dict()
    for lid, ld in linkdata.items():
        splits = lid.split('.')
        if len(splits) <= 1:
            res[lid] = ld
            sublink_map[lid] = lid
        else:
            link_id = splits[0]
            sublink_map[lid] = link_id
    
    return res, sublink_map

linkdata, sublink_map = process_links(linkdata)

class Link:
    '''
    initialize links for finding nearby parking links
    '''
    def __init__(self, idx, parking=None):
        self.id = idx
        self.parking = parking
        # self.len = length

    # def set_inc(self, incomings):
    #     self.inc = incomings
    
    # def set_out(self, outgoings):
    #     self.out = outgoings
    
    def set_neighbors(self, neighbors):
        self.neighbors = neighbors
    
    def find_near_parkings(self, parking_links, num=5):
        '''
        find nearby parking {num} links (excluding itself), default 5 
        '''
        if len(parking_links) <= num + 1:
            return parking_links

        return self.bfs(num)
    
    def bfs(self, num):
        '''
        Breadth-first search (BFS) to find a nearby camera: https://favtutor.com/blogs/breadth-first-search-python
        - create a queue Q 
        - mark v as visited and put v into Q 

        - while Q is non-empty 
            -- remove the head u of Q 
            -- mark and enqueue all (unvisited) neighbors of u
        '''
        cnt = 0
        result = [self.id] # a collection of ids of parking links

        visited = [self]
        queue = [self] # queue of neighbor links to be visited

        while queue:
            cur_link = queue.pop(0)
            for neighbor in cur_link.neighbors:
                if neighbor.parking and neighbor.id not in result:
                    result.append(neighbor.id)
                    cnt += 1 # finish adding 1 more parking lot

                if neighbor not in visited:
                    visited.append(neighbor)
                    queue.append(neighbor)
                
                if cnt > num:
                    result = result[1:] + [result[0]] # put itself at the end of surrounding parking list
                    return result

def init_links(parking_links):
    '''
    Initialize link data
    '''
    links = list(linkdata.keys())
    link_dict = dict()
    link_nearParkings = dict() # store the nearby parking links, default num=5

    # step-1: initialize all link objects
    for link_id in links:
        data = linkdata[link_id]
        if link_id in parking_links:
            parking = link_id
        else:
            parking = None
        
        link_dict[link_id] = Link(link_id, parking)
    
    # step-2: set neighbors of all link objects
    for link_id in links:
        data = linkdata[link_id]
        inc = data['incoming']
        out = data['outgoing']
        
        inc_links = [link_dict[sublink_map[l]] for l in inc] # use sublink_map to mapping sublinks to links
        out_links = [link_dict[sublink_map[l]] for l in out]
        neighbors = inc_links + out_links
        link = link_dict[link_id]
        link.set_neighbors(neighbors)
    
    # step-3: find nearby parkings for the selected parkingLinks
    for link_id in parking_links:
        link = link_dict[link_id]
        link_nearParkings[link_id] = link.find_near_parkings(parking_links, num=5)
    
    return link_dict, link_nearParkings    

def select_parkingLink(all_links, p=.02, num=None):
    '''
    Select parking areas
    '''
    np.random.seed(40)
    select_size = int(p * len(all_links))
    if num:
        select_size = num

    return np.random.choice(all_links, size=select_size, replace=False)

def generate_parkings(parking_links, link_nearParkings, length=1, start_pos=.5, end_pos=.501):
    # use parking space instead of roadsideparking
    # step-1: process parking data
    parkInfos = dict() # {'id': 'length':, 'lane':, 'start_pos':, 'end_pos':}
    parkSpaces = dict()
    capacities = [(12, 10)] * 5 + [(15, 10)] * 10 + [(20, 10)] * 20 + [(16, 10)] * 20 + [(10, 10)] * 30 + [(10, 5)] * 15
    # print('the cap are {}'.format(capacities))

    for link in parking_links:
        nPark = select_capacity(capacities)
        coords = linkdata[link]['coord']
        parkingSpaces = get_parkingSpaces(coords, nPark)
        tmp = {
            'id': 'PA_' + link,
            'lane': link + '_0',
            'length': str(length),
            'start_pos': str(int(start_pos * linkdata[link]['length'])), 
            'end_pos': str(int(end_pos * linkdata[link]['length'])), 
            'roadsideCapacity': '0',
            'friendlyPos': '1',
            'capacity': str(nPark[0] * nPark[1])
        }
        parkInfos[link] = tmp
        parkSpaces[link] = parkingSpaces

    parking2xml(parkInfos, parkSpaces) # write parking data into xml file

    # step-2: process parking rerouter data
    # assuming all parkings are invisible
    begin = '0.0'
    end = str(simLen)

    parkingRR2xml(link_nearParkings, begin, end) # write parking rerouter data into xml files
    
    return parkInfos

def get_parkingSpaces(coords, nPark):
    '''
    get parking spaces
    ：coords: from_node_coord, to_node_coord of the road link
    : nPark: (num_row, num_col) of parking spaces 
    '''
    coords = coords.split()
    from_coords = coords[:2]
    to_coords = coords[2:]
    x = float(to_coords[0]) - float(from_coords[0])
    y = float(to_coords[1]) - float(from_coords[1])
    vec = [x, y, 0]

    norm_vec, orth_vec = _rotate90(vec)
    center_point = [(float(to_coords[0]) + float(from_coords[0])) / 2,
        (float(to_coords[1]) + float(from_coords[1])) / 2]
    
    # print('the centerpoint is {}'.format(center_point))
    parkingSpaces = list()
    
    for r in range(nPark[0]):
        for c in range(nPark[1]):

            parkingSpaces.append(get_spaceXY(center_point, r, c, norm_vec, orth_vec))
    # print('the parkingspace is {}'.format(parkingSpaces))
    return parkingSpaces

def get_spaceXY(o, r, c, vec, orth_vec, offset=4):

    x = o[0] + (offset + c) * orth_vec[0] + r * vec[0]
    y = o[1] + (offset + c) * orth_vec[1] + r * vec[1]
    
    return (x, y)
    
def _rotate90(vec, norm_len=4):
    '''
    get pairwise normalized original and orthogonal vectors
    '''
    rotation_degree = 90
    rotation_radian = np.radians(rotation_degree)
    rotation_axis = np.array([0, 0, -1])
    rotation_vector = rotation_radian * rotation_axis
    rotation = R.from_rotvec(rotation_vector)
    rotated_vector = rotation.apply(vec)

    x, y = int(rotated_vector[0]), int(rotated_vector[1])
    vec_len = np.sqrt(np.square(x) + np.square(y))

    norm_vec = [norm_len * vec[0]/vec_len, norm_len * vec[1]/ vec_len] # normalized original vector
    orth_vec = [norm_len * x/vec_len, norm_len * y/vec_len] # normalized orthogonal vector
    
    return norm_vec, orth_vec

def parking2xml(parking_data, parking_spaces, fp=parking_fp):
    '''
    generate a sumo-readable xml file for links with parkings
    '''

    dom = minidom.getDOMImplementation().createDocument(None,'additional',None)
    root = dom.documentElement
    
    for l, p in parking_data.items():
        parking = dom.createElement('parkingArea')
        [parking.setAttribute(k, v) for k,v in p.items()]
        for sp in parking_spaces[l]:
            space = dom.createElement('space')
            space.setAttribute('x', str(sp[0]))
            space.setAttribute('y', str(sp[1]))
            space.setAttribute('width', '2')
            space.setAttribute('length', '3')
            parking.appendChild(space)
        root.appendChild(parking)

    with open(fp, 'w', encoding='utf-8') as f:
        dom.writexml(f, addindent='\t', newl='\n',encoding='utf-8')

def parkingRR2xml(linkParkings, begin, end, fp=parkingRR_fp):
    '''
    generate a sumo-readable xml file for parking rerouters for each link with parkings
    '''
    dom = minidom.getDOMImplementation().createDocument(None,'additional',None)
    root = dom.documentElement

    for link, nearParking in linkParkings.items():
        # nearParking.append(link) # append the link itself to the nearby parkings
        rr = dom.createElement('rerouter')
        rr.setAttribute('id', 'PA_' + link)
        rr.setAttribute('edges', link)
        interval = dom.createElement('interval')
        interval.setAttribute('begin', begin)
        interval.setAttribute('end', end)
        for nearP in nearParking:
            parkingrr = dom.createElement('parkingAreaReroute')
            parkingrr.setAttribute('id', 'PA_' + nearP)
            parkingrr.setAttribute('visible', 'false')
            
            interval.appendChild(parkingrr)
        rr.appendChild(interval)
        root.appendChild(rr)

    with open(fp, 'w', encoding='utf-8') as f:
        dom.writexml(f, addindent='\t', newl='\n',encoding='utf-8')

    
def select_capacity(capacities):
    # np.random.seed(40)
    selected = np.random.randint(0, 100, size=1)[0]
    # print('the selected is {}'.format(selected))
    return capacities[selected]

def main():
    all_links = list(linkdata.keys())
    parking_links = select_parkingLink(all_links, num=3)

    print('{} parkingLinks are {}'.format(len(parking_links), parking_links))
    
    link_dict, link_nearParkings = init_links(parking_links)
    print(link_nearParkings)
    parking_data = generate_parkings(parking_links, link_nearParkings)

if __name__ == "__main__":
    print('run parking generator')
    main()