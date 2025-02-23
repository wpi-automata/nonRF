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
            k=0.2
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
    agents_async[4].setflagcount(6)

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
   

    plt.ion()
    fig, ax = plt.subplots()
    lmt=200
    ax.set_xlim(-lmt, lmt)
    ax.set_ylim(-lmt, lmt)
    ax.set_title('Consensus Formation Control')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')


    #####formation of agents 
    while count < formation_iterations:
        x = [agent.my_value[0] for agent in agents]  # Include all agents
        y = [agent.my_value[1] for agent in agents]
        x_async = [agent.my_value[0] for agent in agents_async]  # Include all agents
        y_async = [agent.my_value[1] for agent in agents_async]  
       
        # Visualization
        ax.clear()
        
        ax.set_xlim(-lmt, lmt)
        ax.set_ylim(-lmt, lmt)
        ax.scatter(x, y, c='blue', label='Formation Agents')
        ax.scatter(x_async, y_async, c='yellow', label='Formation async Agents')
        ax.legend()
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
    ax.clear() 
    ax.set_xlim(-lmt, lmt)
    ax.set_ylim(-lmt, lmt)
    ax.scatter(x, y, c='blue', label='Formation Agents')
    ax.scatter(x_async, y_async, c='yellow', label='Formation async Agents')
    ax.scatter(target[0], target[1], c='red', label='target')
    ax.scatter(centroid_x, centroid_y, c='gray', label='formation  centre') 
    ax.scatter(centroid_x_async, centroid_y_async, c='green', label='formation  centre') 

    ax.legend()
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
        ax.clear() 
        ax.set_xlim(-lmt, lmt)
        ax.set_ylim(-lmt, lmt)
        ax.scatter(x, y, c='blue', label='Formation Agents')
        ax.scatter(x_async, y_async, c='black', label='Formation async Agents')
        ax.scatter(target[0], target[1], c='red', label='target')
        ax.scatter(centroid_x, centroid_y, c='gray', label='formation  centre')
        ax.scatter(centroid_x_async, centroid_y_async, c='green', label='formation  centre')    
        ax.legend()
        plt.pause(0.1)
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

        ax.clear() 
        # ax.set_xlim(-dynamic_lmt, dynamic_lmt_x)
        # ax.set_ylim(-dynamic_lmt_y, dynamic_lmt_y)
        ax.set_xlim(-lmt, lmt)
        ax.set_ylim(-lmt, lmt)

        # Plot the agents, target, and centroid
        ax.scatter(x, y, c='blue', label='Formation Agents')
        ax.scatter(x_async, y_async, c='black', label='Formation async Agents')
        ax.scatter(target[0], target[1], c='red', label='target')
        ax.scatter(centroid_x, centroid_y, c='gray', label='formation centre')   
        ax.scatter(centroid_x_async, centroid_y_async, c='green', label='formation  centre')
        ax.legend()
        plt.pause(0.5)

    plt.ioff()
    plt.show()


        















    '''
import networkx as nx
from network_agent import DynamicAgent
from distributed_algorithm import run_sim
import matplotlib.pyplot as plt
import random as rand
import math

class ConsensusAgent(DynamicAgent):
    
    def __init__(self, id=0, value=[], check=False, offset=(0,0), velocity=0.1):
        self.my_value = value
        self.id = id
        self.msgs = []
        self.neighbor_sum_x = 0
        self.neighbor_sum_y = 0
        self.rate_x = 0
        self.rate_y = 0
        self.offset = offset  # Stores the formation offset relative to the center
        self.velocity = velocity  # Velocity factor to control the speed of convergence

    def msg(self):
        return (self.id, self.my_value)  # Include sender ID in the message

    def stf(self):
        self.neighbor_sum_x = 0
        self.neighbor_sum_y = 0
        center_x = None
        center_y = None
        
        # Process each message to sum neighbor values and find the center
        for sender_id, value in self.msgs:
            self.neighbor_sum_x += value[0]
            self.neighbor_sum_y += value[1]
            if sender_id == 5:  # ID of the center agent
                center_x = value[0]
                center_y = value[1]
        
        # Calculate the consensus rate
        self.rate_x = self.neighbor_sum_x - (len(self.msgs) * self.my_value[0])
        self.rate_y = self.neighbor_sum_y - (len(self.msgs) * self.my_value[1])
        
        # If center is found, adjust rate to move towards desired position (center + offset)
        if center_x is not None and center_y is not None:
            desired_x = center_x + self.offset[0]
            desired_y = center_y + self.offset[1]
            k = 0.5  # Control gain for formation
            self.rate_x += k * (desired_x - self.my_value[0])
            self.rate_y += k * (desired_y - self.my_value[1])

    def clear_msgs(self):
        self.msgs = []

    def add_msg(self, msg):
        self.msgs.append(msg)

    def total_msg(self, msg):
        value = msg[1]  # Extract the my_value part of the message
        self.neighbor_sum_x += value[0]
        self.neighbor_sum_y += value[1]

    def clear_sum(self):
        self.neighbor_sum_x = 0
        self.neighbor_sum_y = 0

    def step(self):
        self.my_value[0] = round(self.my_value[0] + 0.85*self.velocity * self.rate_x, 3)
        self.my_value[1] = round(self.my_value[1] + 0.85*self.velocity * self.rate_y, 3)


def make_agents(n, list, offsets, velocity):
    agents = []
    for ii in range(n):
        agents.append(ConsensusAgent(ii, list[ii], False, offsets[ii], velocity))
    return agents

def print_max_id(agents):
    for agent in agents:
        print(agent.my_value)

if __name__ == "__main__":
    n = 5
    edge_list = [(0, 1), (1, 2), (2, 3), (3, 4), (4, 0)]
    G = nx.Graph()
    G.add_edges_from(edge_list)
    l = [[4, 15], [1, -3], [-8, 12], [9, 16], [-19, 22]]
    
    # Define pentagon offsets
    radius = 10
    offsets = []
    for i in range(5):
        angle = 2 * math.pi * i / 5
        dx = radius * math.cos(angle)
        dy = radius * math.sin(angle)
        offsets.append((dx, dy))
    
    velocity = 0.1  # Velocity factor to control the speed of convergence
    agents = make_agents(n, l, offsets, velocity)

    print('Initial Values:')
    print_max_id(agents)
    count = 0
    phase = "initial_convergence"
    max_iterations = 500
    convergence_threshold = 1.5  # Increased convergence threshold
    stable_iterations_required = 2  # Number of consecutive iterations required for convergence
    stable_iterations = 0  # Counter for stable iterations
    previous_positions = [agent.my_value[:] for agent in agents]

    plt.ion()
    fig, ax = plt.subplots()
    ax.set_xlim(-25, 25)
    ax.set_ylim(-25, 25)
    ax.set_title('Consensus Formation Control')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')

    # Add new agent as the center
    new_agent = ConsensusAgent(n, [rand.uniform(-25, 25), rand.uniform(-25, 25)], False, (0,0), velocity)
    agents.append(new_agent)
    G.add_node(n)
    for i in range(n):
        G.add_edge(i, n)

    # Store previous iteration's positions
    previous_positions = None
    same_position_count = 0
    
    # Target position for the center node to move to
    target_position = [0, 0]  # Single target
    move_speed = 0.75  # Speed at which center moves towards target
    stay_at_target_iterations = 0  # Counter for staying at the target

    while count < max_iterations:
        x = [agent.my_value[0] for agent in agents[:-1]]
        y = [agent.my_value[1] for agent in agents[:-1]]
        center_x = new_agent.my_value[0]
        center_y = new_agent.my_value[1]

        # Visualization
        ax.clear()
        ax.set_xlim(-25, 25)
        ax.set_ylim(-25, 25)
        ax.scatter(x, y, c='blue', label='Formation Agents')
        ax.scatter(center_x, center_y, c='red', label='Center')
        if phase == "moving":
            target_x, target_y = target_position # Target position for the center
            ax.scatter(target_x, target_y) 
        ax.legend()
        plt.pause(0.1)

        # Store current positions before update
        current_positions = [[round(agent.my_value[0], 3), round(agent.my_value[1], 3)] for agent in agents]

        if phase == "initial_convergence":
            agents = run_sim(agents, G, return_agents=True, ctl=True)
            new_agent.my_value = [center_x, center_y]  # Keep center position constant

            # Check for initial convergence
            if previous_positions is not None:
                positions_match = all(
                    current_positions[i] == previous_positions[i]
                    for i in range(len(agents))
                )
                
                if positions_match:
                    same_position_count += 1
                    if same_position_count >= stable_iterations_required:
                        print(f"Initial convergence achieved at iteration {count}")
                        phase = "moving"
                        same_position_count = 0
                else:
                    same_position_count = 0

        elif phase == "moving":
            # Move center towards target
            target_x, target_y = target_position
            dx = target_x - center_x
            dy = target_y - center_y
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance < 0.20:  # If close enough to target
                stay_at_target_iterations += 1
                if stay_at_target_iterations >= 15:  # Stay at target for 3 iterations
                    print(f"Reached target at iteration {count}")
                    break
            else:
                # Update center position
                new_x = center_x + (dx/distance) * move_speed
                new_y = center_y + (dy/distance) * move_speed
                new_agent.my_value = [new_x, new_y]
            
            # Run formation control
            agents = run_sim(agents, G, return_agents=True, ctl=True)

        print(f'Iteration {count}:')
        print_max_id(agents)
        
        previous_positions = current_positions
        count += 1

    print(f"Simulation completed in {count} iterations")
    nx.draw_circular(G, with_labels=True)
    plt.ioff()
    plt.show()
    '''