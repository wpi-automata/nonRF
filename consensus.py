import networkx as nx
from network_agent import DynamicAgent
from distributed_algorithm import run_sim
import matplotlib.pyplot as plt
import random as rand
import math

class ConsensusAgent(DynamicAgent):
    
    def __init__(self,n, id=0, value=[],vel_target=[],check=False, offset=(0,0), velocity=0.1):
        self.my_value = value
        self.id = id
        self.msgs = []
        self.offset_msgs=[]
        self.neighbor_sum_x = 0
        self.neighbor_sum_y = 0
        self.rate_x = 0
        self.rate_y = 0
        self.offset = offset  # Stores the formation offset relative to the center
        self.velocity = velocity  # Velocity factor to control the speed of convergence
        self.dij_x=0                #the distance vector from this point to the neighbor that means  offset.neighbor - offset.self
        self.dij_y=0 
        self.lm = 1
        self.lo = 5
        self.flagcounter = 0

        ####for consensus on target velocity 
        ############################################
        self.vel_target=vel_target
        self.neighbor_sum_vel_target_x = 0
        self.neighbor_sum_vel_target_y = 0
        self.vel_target_msgs=[]
        ###########################################

    def msg(self):
        return self.my_value  # Include sender ID in the message
    def offset_msg(self):
        return self.offset

    def stf(self):
        # Calculate the consensus rate
        self.flagcounter += 1
        if self.flagcounter >= (self.lm) + (self.lo):
            desired_x=self.dij_x-(len(self.offset_msgs)*self.offset[0])
            desired_y=self.dij_y-(len(self.offset_msgs)*self.offset[1])
            
            self.rate_x = self.neighbor_sum_x - (len(self.msgs) * self.my_value[0])-desired_x
            self.rate_y = self.neighbor_sum_y - (len(self.msgs) * self.my_value[1])-desired_y
            k=0.3
            self.my_value[0] = round(self.my_value[0] + (k * self.rate_x), 3)
            self.my_value[1] = round(self.my_value[1] + (k * self.rate_y), 3)
            self.flagcounter=0

    def clear_msgs(self):
        self.msgs = []
        self.offset_msgs=[]
   
    def add_msg(self, msg):
        self.msgs.append(msg)

    def add_offset_msg(self, offset_msg):
        self.offset_msgs.append(offset_msg)
    
    def setflagcount(self, count):
        self.flagcounter = count

    def total_msg(self, msg,offset_msg):
        value = msg  # Extract the my_value part of the message
        offset_value=offset_msg
        self.neighbor_sum_x += value[0]
        self.neighbor_sum_y += value[1]
        self.dij_x+=offset_value[0]
        self.dij_y+=offset_value[1]

    def clear_sum(self):
        self.neighbor_sum_x = 0
        self.neighbor_sum_y = 0
        self.dij_x=0
        self.dij_y=0

    def step(self):
        pass

    ####consensus on target velocity functions
    ##################################################################################################################
    def vel_target_msg(self):
        return self.vel_target
    
    def vel_target_stf(self):
        # Calculate the consensus rate
        self.flagcounter += 1
        if self.flagcounter >= (self.lm) + (self.lo):
            self.rate_x = self.neighbor_sum_vel_target_x - (len(self.vel_target_msgs) * self.vel_target[0])
            self.rate_y = self.neighbor_sum_vel_target_y - (len(self.vel_target_msgs) * self.vel_target[1])
            k=0.3
            self.vel_target[0] = round(self.vel_target[0] + (k * self.rate_x), 3)
            self.vel_target[1] = round(self.vel_target[1] + (k * self.rate_y), 3)
            self.flagcounter=0

    def clear_vel_target(self):
        self.vel_target_msgs=[]
    
    def add_vel_target_msg(self, msg):
        self.vel_target_msgs.append(msg)
    
    def total_vel_target_msg(self, vel_msg,):
        value = vel_msg  # Extract the my_value part of the message
        self.neighbor_sum_vel_target_x += value[0]
        self.neighbor_sum_vel_target_y += value[1]

    def clear_vel_target_sum(self):
        self.neighbor_sum_vel_target_x=0
        self.neighbor_sum_vel_target_y=0
    ##################################################################################################################
    

def make_agents(n, list, offsets, velocity):
    agents = []
    for ii in range(n):
        agents.append(ConsensusAgent(n,ii, list[ii],[0,0], False, offsets[ii], velocity))
    return agents

def print_max_id(agents):
    for agent in agents:
        print(agent.my_value)

if __name__ == "__main__":
    n = 5
    edge_list = [(0, 1), (1, 2), (2, 3), (3, 4), (4, 0)]
    G = nx.Graph()
    G.add_edges_from(edge_list)
    l = [[100, 15], [1, -312], [-80, 102], [90, 176], [-190, 220]]
    #l1 = [[10, 150], [199, -31], [80, -102], [-90, 16], [90, -220]]
    l1 = [[100, 15], [1, -312], [-80, 102], [90, 176], [-190, 220]]
    # Define pentagon offsets
    radius = 20
    offsets = []
    for i in range(5):
        angle = 2 * math.pi * i / 5
        dx = radius * math.cos(angle)
        dy = radius * math.sin(angle)
        offsets.append((dx, dy))
    
    velocity = 0.1  # Velocity factor to control the speed of convergence
    agents = make_agents(n, l, offsets, velocity)
    agents[0].setflagcount(6)
    agents[1].setflagcount(6)
    agents[2].setflagcount(6)
    agents[3].setflagcount(6)
    agents[4].setflagcount(6)
    agents_async = make_agents(n, l1, offsets, velocity)
    agents_async[0].setflagcount(1)
    agents_async[1].setflagcount(5)
    agents_async[2].setflagcount(3)
    agents_async[3].setflagcount(2)
    agents_async[4].setflagcount(4)

    # agents[0].setflagcount(6)
    # agents[1].setflagcount(6)
    # agents[2].setflagcount(6)
    # agents[3].setflagcount(6)
    # agents[4].setflagcount(6)

    # print('Initial Values:')
    # print_max_id(agents)
    count = 0
    phase = "initial_convergence"
    formation_iterations = 100
    error_list = [] # to plot the error 
   

    plt.ion()
    fig_sync, ax_sync = plt.subplots()
    fig_async, ax_async = plt.subplots() # new figure and axis for the async plot
    fig_error = plt.figure(figsize=(10, 12))  # Create a new figure and axis for the error plot
    ax_error1 = fig_error.add_subplot(3, 1, 1)  # Sync centroid vs target
    ax_error2 = fig_error.add_subplot(3, 1, 2)  # Async centroid vs target 
    ax_error3 = fig_error.add_subplot(3, 1, 3)  # Difference between centroids

    sync_target_errors = []
    async_target_errors = []
    centroid_diff_errors = error_list
    
    #lmt= 1e295
    lmt = 250
    ax_sync.set_xlim(-lmt, lmt)
    ax_sync.set_ylim(-lmt, lmt)
    ax_sync.set_title('Consensus Sync Formation Control')
    ax_sync.set_xlabel('X')
    ax_sync.set_ylabel('Y')

    ax_async.set_xlim(-lmt, lmt)
    ax_async.set_ylim(-lmt, lmt)
    ax_async.set_title('Consensus Async Formation Control')
    ax_async.set_xlabel('X')
    ax_async.set_ylabel('Y')

    ax_error1.set_title('Error between Sync centroids and target over iterations')
    ax_error1.set_xlabel('Iterations')
    ax_error1.set_ylabel('Error')
    ax_error1.legend()

    ax_error2.set_title('Error between Async centroids and target over iterations')
    ax_error2.set_xlabel('Iterations')
    ax_error2.set_ylabel('Error')
    ax_error2.legend()

    ax_error3.set_title('Error between Sync and Async centroids over iterations')
    ax_error3.set_xlabel('Iterations')
    ax_error3.set_ylabel('Error')
    ax_error3.legend()

    plt.pause(0.1)


    #####formation of agents 
    while count < formation_iterations:
        x = [agent.my_value[0] for agent in agents]  # Include all agents
        y = [agent.my_value[1] for agent in agents]
        x_async = [agent.my_value[0] for agent in agents_async]  # Include all agents
        y_async = [agent.my_value[1] for agent in agents_async]  
       
        # Visualization
        ax_sync.clear()
        
        ax_sync.set_xlim(-lmt, lmt)
        ax_sync.set_ylim(-lmt, lmt)
        ax_sync.scatter(x, y, c='blue', label='Formation Agents')
        ax_sync.legend()

        ax_async.clear()
        ax_async.set_xlim(-lmt, lmt)
        ax_async.set_ylim(-lmt, lmt)
        ax_async.scatter(x_async, y_async, c='black', label='Formation async Agents')
        ax_async.legend()

        plt.pause(0.1)

        if phase == "initial_convergence":
            agents = run_sim(agents, G, return_agents=True, ctl=True,target_pos=False)
            agents_async = run_sim(agents_async, G, return_agents=True, ctl=True,target_pos=False)
        
        count += 1

    print(f"Simulation completed in {count} iterations")
    
    
   

    ###gethering around the target
    target=[0,0]
    centroid_x=sum(a.my_value[0] for a in agents)/len(agents)
    centroid_y=sum(a.my_value[1] for a in agents)/len(agents)
    centroid_x_async=sum(a.my_value[0] for a in agents_async)/len(agents_async)
    centroid_y_async=sum(a.my_value[1] for a in agents_async)/len(agents_async)
    
    
    # Update synchronous plot
    ax_sync.clear() 
    ax_sync.set_xlim(-lmt, lmt)
    ax_sync.set_ylim(-lmt, lmt)
    ax_sync.scatter(x, y, c='blue', label='Formation Agents')
    ax_sync.scatter(target[0], target[1], c='red', label='Target')
    ax_sync.scatter(centroid_x, centroid_y, c='gray', label='Formation Centre') 
    ax_sync.grid(True)
    ax_sync.legend()

    # Update asynchronous plot
    ax_async.clear() 
    ax_async.set_xlim(-lmt, lmt)
    ax_async.set_ylim(-lmt, lmt)
    ax_async.scatter(x_async, y_async, c='black', label='Formation Async Agents')
    ax_async.scatter(target[0], target[1], c='red', label='Target')
    ax_async.scatter(centroid_x_async, centroid_y_async, c='green', label='Formation Centre')
    ax_async.grid(True)
    ax_async.legend()

      


    plt.pause(0.1)
    print("value of centroid synchronous",centroid_x,centroid_y)
    print("value of centroid asynchronous",centroid_x_async,centroid_y_async)
    ##lets it go over it in 10 iterations
    cover_iterations=10
    dx_per_iter=(target[0]-centroid_x)/cover_iterations
    dy_per_iter=(target[1]-centroid_y)/cover_iterations
    async_dx_per_iter=(target[0]-centroid_x_async)/cover_iterations
    async_dy_per_iter=(target[1]-centroid_y_async)/cover_iterations
    count=0
    while count<cover_iterations:
        for agent in agents:
            agent.my_value[0]+=dx_per_iter
            agent.my_value[1]+=dy_per_iter
        for agent in agents_async:
            agent.my_value[0]+=async_dx_per_iter
            agent.my_value[1]+=async_dy_per_iter
        x = [agent.my_value[0] for agent in agents]  
        y = [agent.my_value[1] for agent in agents]
        x_async = [agent.my_value[0] for agent in agents_async]  # Include all agents
        y_async = [agent.my_value[1] for agent in agents_async] 
        centroid_x=sum(a.my_value[0] for a in agents)/len(agents)
        centroid_y=sum(a.my_value[1] for a in agents)/len(agents)
        centroid_x_async=sum(a.my_value[0] for a in agents_async)/len(agents_async)
        centroid_y_async=sum(a.my_value[1] for a in agents_async)/len(agents_async)

       # Update synchronous plot
        ax_sync.clear() 
        ax_sync.set_xlim(-lmt, lmt)
        ax_sync.set_ylim(-lmt, lmt)
        ax_sync.scatter(x, y, c='blue', label='Formation Agents')
        ax_sync.scatter(target[0], target[1], c='red', label='Target')
        ax_sync.scatter(centroid_x, centroid_y, c='gray', label='Formation Centre')
        ax_sync.grid(True)
        ax_sync.legend()

        # Update asynchronous plot
        ax_async.clear() 
        ax_async.set_xlim(-lmt, lmt)
        ax_async.set_ylim(-lmt, lmt)
        ax_async.scatter(x_async, y_async, c='black', label='Formation Async Agents')
        ax_async.scatter(target[0], target[1], c='red', label='Target')
        ax_async.scatter(centroid_x_async, centroid_y_async, c='green', label='Formation Centre')
        ax_async.grid(True)
        ax_async.legend()
        
        count+=1
    
    #####move as per motion of target
    ###start simple with giving motion to target
    ####another while loop to consensus on velocity
    test_iterations = 1000
    count = 0
    prev_x = target[0]
    prev_y = target[1]

    while count < test_iterations:
        target[0] = prev_x + rand.randint(-12, 13)  # Use randint for integers
        target[1] = prev_y + rand.randint(-12, 13)  # Use randint for integers
        count += 1  # Increment count to avoid infinite loop

        for agent,agent_async in zip(agents,agents_async):
            err = rand.randint(-11, 11) / 100
            agent.vel_target[0] = ((1 - err) * target[0]) - prev_x
            agent.vel_target[1] = ((1 - err) * target[1]) - prev_y
            agent_async.vel_target[0] = ((1 - err) * target[0]) - prev_x
            agent_async.vel_target[1] = ((1 - err) * target[1]) - prev_y

        inner_count = 0
        while inner_count < 5000:  # Inner loop to check velocity differences
            # Check if all agents meet the condition to break out of inner loop
            if (all(abs(agent.vel_target[0] - target[0]) < 3 and ####if both of them have converged before 100
             abs(agent.vel_target[1] - target[1]) < 3
             for agent in agents) and
        all(abs(agent_async.vel_target[0] - target[0]) < 10 and 
            abs(agent_async.vel_target[1] - target[1]) < 10
            for agent_async in agents_async)):
                break  

            # Run simulation and update agents
            agents = run_sim(agents, G, return_agents=True, ctl=True, target_pos=True)
            agents_async = run_sim(agents_async, G, return_agents=True, ctl=True, target_pos=True)
            inner_count += 1
        for agent in agents:
            print("agent value",agent.vel_target)
            print("flag value",agent.flagcounter)

        for agent in agents_async:
            print("agent_async value",agent.vel_target)
            print("flag value",agent.flagcounter)


        # Consensus of velocity targets

        # consensus_vx = sum(agent.vel_target[0] for agent in agents) / len(agents)
        # consensus_vy = sum(agent.vel_target[1] for agent in agents) / len(agents)
        # consensus_vx_async = sum(agent_async.vel_target[0] for agent_async in agents_async) / len(agents_async)
        # consensus_vy_async = sum(agent_async.vel_target[1] for agent_async in agents_async) / len(agents_async)

        consensus_vx = agents[0].vel_target[0]
        consensus_vy = agents[0].vel_target[1]
        consensus_vx_async = agents_async[0].vel_target[0]
        consensus_vy_async = agents_async[0].vel_target[1]
        prev_x, prev_y = target

        for agent in agents:
            agent.my_value[0] += consensus_vx
            agent.my_value[1] += consensus_vy

        for agent in agents_async:
            agent.my_value[0] += consensus_vx_async
            agent.my_value[1] += consensus_vy_async

        x = [agent.my_value[0] for agent in agents]  
        y = [agent.my_value[1] for agent in agents]
        x_async = [agent.my_value[0] for agent in agents_async]  # Include all agents
        y_async = [agent.my_value[1] for agent in agents_async] 
        
        # Calculate the centroid
        centroid_x = sum(a.my_value[0] for a in agents) / len(agents)
        centroid_y = sum(a.my_value[1] for a in agents) / len(agents)
        centroid_x_async=sum(a.my_value[0] for a in agents_async)/len(agents_async)
        centroid_y_async=sum(a.my_value[1] for a in agents_async)/len(agents_async)
        
        # Calculate dynamic limits based on target and centroid positions
        # dynamic_lmt_x = max(abs(target[0]), abs(centroid_x)) + lmt
        # dynamic_lmt_y = max(abs(target[1]), abs(centroid_y)) + lmt

        # Calculate the error between the centroids
        error = math.sqrt((centroid_x - centroid_x_async) ** 2 + (centroid_y - centroid_y_async) ** 2)
        error_list.append(error)

        
        # Update synchronous plot
        ax_sync.clear() 
        ax_sync.set_xlim(-lmt, lmt)
        ax_sync.set_ylim(-lmt, lmt)
        ax_sync.scatter(x, y, c='blue', label='Formation Agents')
        ax_sync.scatter(target[0], target[1], c='red', label='Target')
        ax_sync.scatter(centroid_x, centroid_y, c='gray', label='Formation Centre')
        ax_sync.grid(True)
        ax_sync.legend()

        # Update asynchronous plot
        ax_async.clear() 
        ax_async.set_xlim(-lmt, lmt)
        ax_async.set_ylim(-lmt, lmt)
        ax_async.scatter(x_async, y_async, c='black', label='Formation Async Agents')
        ax_async.scatter(target[0], target[1], c='red', label='Target')
        ax_async.scatter(centroid_x_async, centroid_y_async, c='green', label='Formation Centre')
        ax_async.grid(True)
        ax_async.legend()

        plt.pause(0.5)

        # Calculate errors
        sync_target_error = math.sqrt((centroid_x - target[0])**2 + (centroid_y - target[1])**2)
        async_target_error = math.sqrt((centroid_x_async - target[0])**2 + (centroid_y_async - target[1])**2)
        sync_target_errors.append(sync_target_error)
        async_target_errors.append(async_target_error)
        
        # Update all three error plots
        ax_error1.clear()
        ax_error1.plot(range(len(sync_target_errors)), sync_target_errors, 'b-', label='Sync Centroid vs Target')
        ax_error1.set_title('Synchronous Centroid Distance from Target')
        ax_error1.set_xlabel('Iterations')
        ax_error1.set_ylabel('Error')
        ax_error1.grid(True)
        ax_error1.legend()

        ax_error2.clear()
        ax_error2.plot(range(len(async_target_errors)), async_target_errors, 'r-', label='Async Centroid vs Target')
        ax_error2.set_title('Asynchronous Centroid Distance from Target')
        ax_error2.set_xlabel('Iterations')
        ax_error2.set_ylabel('Error')
        ax_error2.grid(True)
        ax_error2.legend()

        ax_error3.clear()
        ax_error3.plot(range(len(error_list)), error_list, 'g-', label='Difference between Centroids')
        ax_error3.set_title('Error between Sync and Async Centroids')
        ax_error3.set_xlabel('Iterations')
        ax_error3.set_ylabel('Error')
        ax_error3.grid(True)
        ax_error3.legend()

        # Adjust layout to prevent overlap
        plt.tight_layout()
        plt.pause(0.1)


    plt.ioff()
    plt.show()
    fig_error.show()  # Show the error plot
