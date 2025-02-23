'''
import networkx as nx

from network_agent import DynamicAgent
from distributed_algorithm import run_sim
import matplotlib.pyplot as plt
import random as rand
import time

class ConsensusAgent(DynamicAgent):
   
    def __init__(self,id,value=[],check=False):
        self.my_value =value
        self.id=id
        self.msgs = []
        self.neighbor_sum_x=0
        self.neighbor_sum_y=0
        self.rate_x=0
        self.rate_y=0
        self.lm=1
        self.lo=5
        self.flagcounter=0
    def msg(self):
        return self.my_value
       
   
    def stf(self):
        x_form=0
        y_form=0
        if self.id==0:
            x_form=-8
            y_form=-8
        elif self.id==1:
            x_form=-8
            y_form=8
        elif self.id==2:
            x_form=8
            y_form=-8
        elif self.id==3:
            x_form=8
            y_form=8
        self.flagcounter+=1
        if self.flagcounter>=(self.lm)+(self.lo):
            self.rate_x=self.neighbor_sum_x - (len(self.msgs)*self.my_value[0])+x_form
            self.rate_y=self.neighbor_sum_y - (len(self.msgs)*self.my_value[1])+y_form
            self.my_value[0]=round(self.my_value[0] + (0.60*self.rate_x),3)
            self.my_value[1]=round(self.my_value[1] + (0.60*self.rate_y),3)
            self.flagcounter=0
       

    def clear_msgs(self):
        self.msgs = []
   
    def clear_sum(self):
        self.neighbor_sum_x = 0
        self.neighbor_sum_y=0

    def setflagcount(self,count):
        self.flagcounter=count

    def add_msg(self,msg):
        self.msgs.append(msg)
   
    def total_msg(self,msg):
        self.neighbor_sum_x=self.neighbor_sum_x+msg[0]
        self.neighbor_sum_y=self.neighbor_sum_y+msg[1]
 
    def step(self):
        pass
 

def make_agents(n,list):
    agents = []

    for ii in range(n):
        agents.append(ConsensusAgent(ii,list[ii],False))

    return agents

def print_max_id(agents):
    for agent in agents:
        print(agent.my_value)

if __name__=="__main__":
   
    # number of agents
   
    n=4
    #********************Uncomment one the two lines given below to see convergence for different number of edges**********

    #**********make sure only 1 line is uncommented*******
    edge_list=[(0,1),(0,2),(0,3),(1,3),(1,2),(2,3)]
    #edge_list=[(0,1),(0,2),(0,3),(1,3),(3,4),(1,2),(2,4)]


   #*********************************************************************************************************************

    # generate a connected graph
    G = nx.Graph()
    G.add_edges_from(edge_list)
   
   
   
   
   
    l=[[4,15],[1,-3],[-8,12],[9,16]]
   
    # initialize agents
    agents = make_agents(n,l)
    agents[0].setflagcount(6)
    agents[1].setflagcount(3)
    agents[2].setflagcount(2)
    agents[3].setflagcount(1)
    #Print max_id of all agents
    print('Initial Value: ')
    print_max_id(agents)
    count=0
    condition=True
    # run the simulation
    # run_sim(agents,G,diam) # uncomment to run sim without returning agents
    while condition:
        x=[agent.my_value[0] for agent in agents ]
        xmin=min(x)
        xmax=max(x)

        y=[agent.my_value[1] for agent in agents ]
        ymin=min(y)
        ymax=max(y)

        fig, members = plt.subplots()
       
# Plot data
        members.plot(x, y,linestyle='None', marker='o', markersize=10)

# Set axis limits explicitly to [0, 25]
        members.set_xlim([-100, 100])
        members.set_ylim([-100,100])
       
# Add labels and title
        members.set_xlabel('X-axis')
        members.set_ylabel('Y-axis')
        members.set_title('ConsensuProtocol')
        plt.draw()
        plt.pause(1)
        agents = run_sim(agents,G,return_agents=True,ctl=True) # uncomment to run sim and return the agents

    # Print max_id of all agents
        print('New Value: ')
        print_max_id(agents)
        count+=1
        # if xmax-xmin<1 and ymax-ymin<1:
        if count>50:
            condition=False
        plt.close(fig)
       
    print("Final count is: ")
    print(count)
    nx.draw_spring(G ,with_labels=True)
    plt.show()
    '''
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
            self.my_value[0] = round(self.my_value[0] + (0.4 * self.rate_x), 3)
            self.my_value[1] = round(self.my_value[1] + (0.4 * self.rate_y), 3)
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
   
    l = [[45,15],[-11,60],[-8,-55],[9,16]]
  

    agents = make_agents(n, l)
    agents[0].setflagcount(6)
    agents[1].setflagcount(3)
    agents[2].setflagcount(2)
    agents[3].setflagcount(1)
   
    
   
    print('Initial Value: ')
    print_max_id(agents)
   
    fig, members = plt.subplots()
    members.set_xlim([-25, 25])
    members.set_ylim([-25, 25])
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


