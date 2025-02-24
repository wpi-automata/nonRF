import networkx as nx
import matplotlib.pyplot as plt
import random as rand
import time
from network_agent import DynamicAgent
from distributed_algo import run_sim
from matplotlib.animation import FuncAnimation
import math

class ConsensusAgent(DynamicAgent):
   
    def __init__(self, id, value=[]):
        self.my_value = value
        self.id = id
        self.msgs = []
        self.neighbor_sum_x = 0
        self.neighbor_sum_y = 0
        self.rate_x = 0
        self.rate_y = 0
        self.lm = 1
        self.lo = 5
       
        self.flagcounter = 0

    def msg(self):
        return self.my_value
   
    def stf(self):
        d=8
        x_form = 0
        y_form = 0
        if self.id == 0:
            x_form = -d
            y_form = -d
        elif self.id == 1:
            x_form = -d
            y_form = d
        elif self.id == 2:
            x_form = d
            y_form = -d
        elif self.id == 3:
            x_form = d
            y_form = d
        self.flagcounter += 1
        if self.flagcounter >= (self.lm) + (self.lo):
            self.rate_x = self.neighbor_sum_x - (len(self.msgs) * self.my_value[0]) + x_form
            self.rate_y = self.neighbor_sum_y - (len(self.msgs) * self.my_value[1]) +y_form
            self.my_value[0] = round(self.my_value[0] + (0.3 * self.rate_x), 3)
            self.my_value[1] = round(self.my_value[1] + (0.3 * self.rate_y), 3)
            self.flagcounter = 0
           
    def clear_msgs(self):
        self.msgs = []
   
    def clear_sum(self):
        self.neighbor_sum_x = 0
        self.neighbor_sum_y = 0

    def setflagcount(self, count):
        self.flagcounter = count

    def add_msg(self, msg):
        self.msgs.append(msg)
   
    def total_msg(self, msg):
        self.neighbor_sum_x += msg[0]
        self.neighbor_sum_y += msg[1]
   
    def step(self):
        pass
 
def make_agents(n, list):
    return [ConsensusAgent(ii, list[ii]) for ii in range(n)]

def print_max_id(agents):
    for agent in agents:
        print(agent.my_value)

if __name__ == "__main__":
    n = 4
    edge_list = [(0,1),(0,2),(0,3),(1,3),(1,2),(2,3)]
    G = nx.Graph()
    G.add_edges_from(edge_list)
   
    l = [[90,90],[0,90],[90,15],[5,5]]
  

    agents = make_agents(n, l)
    agents[0].setflagcount(6)
    agents[1].setflagcount(3)
    agents[2].setflagcount(2)
    agents[3].setflagcount(1)
   
    
   
    print('Initial Value: ')
    print_max_id(agents)
   
    fig, members = plt.subplots()
    lmt= 50
    members.set_xlim([0, lmt])
    members.set_ylim([0, lmt])
    members.set_xlabel('X-axis')
    members.set_ylabel('Y-axis')
    members.set_title('Consensus Protocol')
   
    scatter = members.scatter([agent.my_value[0] for agent in agents],
                              [agent.my_value[1] for agent in agents],
                              c='blue', s=100)
   
    def update(frame):
        global agents
        agents = run_sim(agents, G, return_agents=True, ctl=True)
        scatter.set_offsets([[agent.my_value[0], agent.my_value[1]] for agent in agents])
        print('New Value: ')
        print_max_id(agents)
        return scatter,
   
    ani = FuncAnimation(fig, update, frames=50, interval=500, repeat=False)
    plt.show()
   
    print("Final count is: ", 50)
    nx.draw_spring(G, with_labels=True)
    plt.show()


