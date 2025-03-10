import networkx as nx
from network_agent import DynamicAgent
from distributed_algorithm import run_sim
import matplotlib.pyplot as plt
import numpy as np
import random as rand
import math
from utilities import update_graph_with_fov, save_frame, draw_fov, check_in_fov, modified_fov_adaptation, check_minimum_connectivity, recover_isolated_agents
from utilities import calculate_fov_direction, update_agent_fov_directions
#FoV Parameters
FOV_ANGLE = 144 # Field of view angle in degrees (equivalent to 72 degrees on each side)
FOV_RANGE = 500 # Maximum range of the FOV in  meters

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
        self.fov_direction = 0  # Current FOV bearing in radians
        self.neighbor_count = 0  # Track number of visible neighbors

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
            #-----------------
            # Calculate target-relative position
            # target_vector = np.array(target) - np.array(self.my_value)
            # target_influence = 0.2 * target_vector
            
            # # Calculate neighbor-relative positions
            # neighbor_influence = np.array([0.0, 0.0])
            # if self.msgs:
            #     avg_neighbor_pos = np.array([
            #         sum(p[0] for p in self.msgs)/len(self.msgs),
            #         sum(p[1] for p in self.msgs)/len(self.msgs)
            #     ])
            #     neighbor_influence = 0.8 * (avg_neighbor_pos - self.my_value + self.offset)
            
            # # Combined update
            # total_influence = target_influence + neighbor_influence
            # self.my_value += self.velocity * total_influence
            # self.flagcounter = 0
            #------------------
            # Calculate target-relative position
            # target_vector = np.array(target) - np.array(self.my_value)
            # target_influence = 0.2 * target_vector

            # # Calculate neighbor-relative positions
            # neighbor_influence = np.array([0.0, 0.0])
            # if self.msgs:
            #     avg_neighbor_pos = np.array([
            #         sum(p[0] for p in self.msgs) / len(self.msgs),
            #         sum(p[1] for p in self.msgs) / len(self.msgs)
            #     ])
            #     neighbor_influence = 0.8 * (avg_neighbor_pos - self.my_value + self.offset)
            # 
            # # Combined influence
            # total_influence = target_influence + neighbor_influence

            # # Explicit velocity control update
            # k = 0.3  # Explicit control gain
            # self.my_value[0] = round(self.my_value[0] + (k * total_influence[0]), 3)
            # self.my_value[1] = round(self.my_value[1] + (k * total_influence[1]), 3)

            # self.flagcounter = 0

            

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
    def scan_for_neighbors(self, scan_speed=0.1):
        """Rotate FOV to scan for neighbors when isolated"""
        self.fov_direction += scan_speed
        if self.fov_direction > math.pi:
            self.fov_direction -= 2 * math.pi

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

    # Define the formation control parameters
    formation_threshold = 0.1  # Threshold for formation convergence
    prev_positions = None
    stable_count = 0
    required_stable_iterations = 10  # Number of iterations positions must remain stable

    

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
    fig = plt.figure(figsize=(15, 12))
    gs = plt.GridSpec(2, 3, figure=fig)  # Create a 2x3 grid

    # Create subplots with specific grid positions
    ax_sync = fig.add_subplot(gs[0, 0])    # Top left
    ax_async = fig.add_subplot(gs[0, 2])      # Top right
    ax_error1 = fig.add_subplot(gs[1, 0])     # Bottom left
    ax_error2 = fig.add_subplot(gs[1, 1])     # Bottom middle
    ax_error3 = fig.add_subplot(gs[1, 2])     # Bottom right

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

    ax_error1.set_title('Error b/w Sync centroids and target over iterations')
    ax_error1.set_xlabel('Iterations')
    ax_error1.set_ylabel('Error')
    ax_error1.legend()

    ax_error2.set_title('Error b/w Async centroids and target over iterations')
    ax_error2.set_xlabel('Iterations')
    ax_error2.set_ylabel('Error')
    ax_error2.legend()

    ax_error3.set_title('Error b/w Sync and Async centroids over iterations')
    ax_error3.set_xlabel('Iterations')
    ax_error3.set_ylabel('Error')
    ax_error3.legend()

    plt.pause(0.1)


    target = [0, 0]  # Initialize target position

    #####formation of agents 
    while count < formation_iterations:
        x = [agent.my_value[0] for agent in agents]  # Include all agents
        y = [agent.my_value[1] for agent in agents]
        x_async = [agent.my_value[0] for agent in agents_async]  # Include all agents
        y_async = [agent.my_value[1] for agent in agents_async]  # Include all agents
        
       
        if count % 1 == 0:  # Every iteration
            # Track connectivity stats
            num_edges = G.number_of_edges()
            avg_degree = sum(dict(G.degree()).values()) / len(agents)
            isolated_agents = check_minimum_connectivity(agents, G)
    
        # Log statistics
        print(f"Iteration {count}: Edges={num_edges}, Avg Degree={avg_degree:.2f}, Isolated={len(isolated_agents)}")
        
        # Apply recovery only to completely isolated agents
        if isolated_agents:
            recover_isolated_agents(agents, isolated_agents, target)


        target = [target[0], target[1]]  # Create a list for target position  # Define target position
        # Get current positions
        current_positions = [(agent.my_value[0], agent.my_value[1]) for agent in agents]
        current_positions_async = [(agent.my_value[0], agent.my_value[1]) for agent in agents_async]
        # Check if positions have stabilized
        if prev_positions is not None:
            # Calculate maximum movement of any agent
            max_movement = max(math.sqrt((curr[0] - prev[0])**2 + (curr[1] - prev[1])**2)
                            for curr, prev in zip(current_positions, prev_positions))
            max_movement_async = max(math.sqrt((curr[0] - prev[0])**2 + (curr[1] - prev[1])**2)
                                for curr, prev in zip(current_positions_async, prev_positions_async))
            
            if max_movement < formation_threshold and max_movement_async < formation_threshold:
                stable_count += 1
            else:
                stable_count = 0
                
            # Break if formation has been stable for enough iterations
            if stable_count >= required_stable_iterations:
                print(f"Formation converged after {count} iterations")
                break
        
        prev_positions = current_positions
        prev_positions_async = current_positions_async
        
        # Update FOV directions
        update_agent_fov_directions(agents, target, neighbors_only=True)
        update_agent_fov_directions(agents_async, target, neighbors_only=True)
        # Draw FOV for sync agents
        for agent in agents:
            draw_fov(ax_sync, agent.my_value, agent.fov_direction, 
                    math.radians(FOV_ANGLE), FOV_RANGE, 'blue', 0.1)
        
        # Draw FOV for async agents
        for agent in agents_async:
            draw_fov(ax_async, agent.my_value, agent.fov_direction, 
                    math.radians(FOV_ANGLE), FOV_RANGE, 'red', 0.1)

        
        # Update graph with FOV constraints

        update_graph_with_fov(agents, agents_async,target, G, fov_angle=FOV_ANGLE, max_range=FOV_RANGE) # Update graph with FOV constraints
        # Check if graph is connected enough for formation control
        if not nx.is_connected(G):
            print(f"Warning: Graph not fully connected at iteration {count}")
        

        # Override FOV direction to face outward (pentagon offset direction)
        for i, agent in enumerate(agents):
            # Get neighbor positions (excluding self)
            neighbor_positions = [a.my_value for j, a in enumerate(agents) if j != i]
            
            # Calculate dynamic FOV direction
            agent.fov_direction = calculate_fov_direction(agent, target, neighbor_positions)
                    
            # Set FOV direction based on formation offset
            fov_direction = math.atan2(agent.offset[1], agent.offset[0])
            
            # Update edges based on mutual visibility
            for j, other_agent in enumerate(agents):
                if i == j:
                    continue
                    
                # Bidirectional visibility check
                sees_other = check_in_fov(agent.my_value, other_agent.my_value,
                                        agent.fov_direction, math.radians(FOV_ANGLE), FOV_RANGE)
                seen_by_other = check_in_fov(other_agent.my_value, agent.my_value,
                                        other_agent.fov_direction, math.radians(FOV_ANGLE), FOV_RANGE)
                
                if sees_other and seen_by_other:
                    G.add_edge(i, j)
                    agent.neighbor_count += 1
                elif G.has_edge(i, j):
                    G.remove_edge(i, j)
                    agent.neighbor_count = max(0, agent.neighbor_count-1)

        x = [agent.my_value[0] for agent in agents]
        y = [agent.my_value[1] for agent in agents]
        # Visualization
        ax_sync.clear()
        
        ax_sync.set_xlim(-lmt, lmt)
        ax_sync.set_ylim(-lmt, lmt)



        ax_sync.legend()

        ax_async.clear()
        ax_async.set_xlim(-lmt, lmt)
        ax_async.set_ylim(-lmt, lmt)



        ax_async.legend()

        # Draw FOV for sync agents
        for agent in agents:
            fov_direction = math.atan2(target[1] - agent.my_value[1], 
                                    target[0] - agent.my_value[0])
            draw_fov(ax_sync, agent.my_value, fov_direction, 
                    math.radians(FOV_ANGLE), FOV_RANGE, 'red', 0.1)
        # # Draw FOV for sync agents
        # for agent in agents:
        #     other_positions = [[a.my_value[0], a.my_value[1]] for a in agents if a.id != agent.id]
        #     fov_direction = math.atan2(agent.offset[1], agent.offset[0])
        #     draw_fov(ax_sync, agent.my_value, fov_direction, 
        #             math.radians(FOV_ANGLE), FOV_RANGE, 'blue', 0.1)
        # Draw FOV for async agents
        for agent in agents_async:
            fov_direction = math.atan2(target[1] - agent.my_value[1], 
                                    target[0] - agent.my_value[0])
            draw_fov(ax_async, agent.my_value, fov_direction, 
                    math.radians(FOV_ANGLE), FOV_RANGE, 'red', 0.1)
        # Draw FOV for async agents
        # for agent in agents_async:
        #     other_positions = [[a.my_value[0], a.my_value[1]] for a in agents_async if a.id != agent.id]
        #     fov_direction = math.atan2(agent.offset[1], agent.offset[0])
        #     draw_fov(ax_async, agent.my_value, fov_direction, 
        #             math.radians(FOV_ANGLE), FOV_RANGE, 'red', 0.1)
        
        # Draw communication links based on graph edges
        for edge in G.edges():
            # For synchronous agents
            if edge[0] < len(agents) and edge[1] < len(agents):
                ax_sync.plot([agents[edge[0]].my_value[0], agents[edge[1]].my_value[0]],
                            [agents[edge[0]].my_value[1], agents[edge[1]].my_value[1]],
                            'b--', alpha=0.3)
            # For asynchronous agents
            elif edge[0] >= len(agents) and edge[1] >= len(agents):
                i, j = edge[0]-len(agents), edge[1]-len(agents)
                ax_async.plot([agents_async[i].my_value[0], agents_async[j].my_value[0]],
                            [agents_async[i].my_value[1], agents_async[j].my_value[1]],
                            'r--', alpha=0.3)
        
        # # Draw communication links based on FOV visibility
        # for i, agent1 in enumerate(agents):
        #     for j, agent2 in enumerate(agents):
        #         if i != j:
        #             # Check if agents are in each other's FOV
        #             if (check_in_fov(agent1.my_value, agent2.my_value, agent1.fov_direction, 
        #                             math.radians(FOV_ANGLE), FOV_RANGE) and
        #                 check_in_fov(agent2.my_value, agent1.my_value, agent2.fov_direction, 
        #                             math.radians(FOV_ANGLE), FOV_RANGE)):
        #                 ax_sync.plot([agent1.my_value[0], agent2.my_value[0]],
        #                         [agent1.my_value[1], agent2.my_value[1]],
        #                         'b--', alpha=0.3)

        # # Draw communication links for async agents
        # for i, agent1 in enumerate(agents_async):
        #     for j, agent2 in enumerate(agents_async):
        #         if i != j:
        #             # Check if agents are in each other's FOV
        #             if (check_in_fov(agent1.my_value, agent2.my_value, agent1.fov_direction, 
        #                             math.radians(FOV_ANGLE), FOV_RANGE) and
        #                 check_in_fov(agent2.my_value, agent1.my_value, agent2.fov_direction, 
        #                             math.radians(FOV_ANGLE), FOV_RANGE)):
        #                 ax_async.plot([agent1.my_value[0], agent2.my_value[0]],
        #                             [agent1.my_value[1], agent2.my_value[1]],
        #                             'r--', alpha=0.3)
                
        # Update synchronous plot         
        ax_sync.scatter(x, y, c='blue', label='Formation Agents')
        # Update asynchronous plot
        ax_async.scatter(x_async, y_async, c='black', label='Formation async Agents')


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
    # Draw FOV for sync agents
    for agent in agents:
        other_positions = [[a.my_value[0], a.my_value[1]] for a in agents if a.id != agent.id]
        fov_direction = math.atan2(agent.offset[1], agent.offset[0])
        draw_fov(ax_sync, agent.my_value, fov_direction, 
                math.radians(FOV_ANGLE), FOV_RANGE, 'blue', 0.1)
    
   
    ax_sync.legend()

    # Update asynchronous plot
    ax_async.clear() 
    ax_async.set_xlim(-lmt, lmt)
    ax_async.set_ylim(-lmt, lmt)
    ax_async.scatter(x_async, y_async, c='black', label='Formation Async Agents')
    ax_async.scatter(target[0], target[1], c='red', label='Target')
    ax_async.scatter(centroid_x_async, centroid_y_async, c='red', label='Formation Centre')
    ax_async.grid(True)
    #  # Draw FOV for async agents
    # for agent in agents_async:
    #     fov_direction = math.atan2(target[1] - agent.my_value[1], 
    #                             target[0] - agent.my_value[0])
    #     draw_fov(ax_async, agent.my_value, fov_direction, 
    #             math.radians(FOV_ANGLE), FOV_RANGE, 'red', 0.1)
    # Draw FOV for async agents
    for agent in agents_async:
        other_positions = [[a.my_value[0], a.my_value[1]] for a in agents_async if a.id != agent.id]
        fov_direction = math.atan2(agent.offset[1], agent.offset[0])
        draw_fov(ax_async, agent.my_value, fov_direction, 
                math.radians(FOV_ANGLE), FOV_RANGE, 'red', 0.1)
    
    ax_async.legend()

      

    update_graph_with_fov(agents, agents_async, target, G, fov_angle=FOV_ANGLE, max_range=FOV_RANGE)
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
    test_iterations = 100
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
             for agent in agents) and all(abs(agent_async.vel_target[0] - target[0]) < 10 and 
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
        # Draw FOV for sync agents
        for agent in agents:
            fov_direction = math.atan2(target[1] - agent.my_value[1], 
                                    target[0] - agent.my_value[0])
            draw_fov(ax_sync, agent.my_value, fov_direction, 
                    math.radians(FOV_ANGLE), FOV_RANGE, 'red', 0.1)
        # Draw FOV for sync agents
        # for agent in agents:
        #     other_positions = [[a.my_value[0], a.my_value[1]] for a in agents if a.id != agent.id]
        #     fov_direction = math.atan2(agent.offset[1], agent.offset[0])
        #     draw_fov(ax_sync, agent.my_value, fov_direction, 
        #             math.radians(FOV_ANGLE), FOV_RANGE, 'blue', 0.1)

        ax_sync.legend()

        # Update asynchronous plot
        ax_async.clear() 
        ax_async.set_xlim(-lmt, lmt)
        ax_async.set_ylim(-lmt, lmt)
        ax_async.scatter(x_async, y_async, c='black', label='Formation Async Agents')
        ax_async.scatter(target[0], target[1], c='red', label='Target')
        ax_async.scatter(centroid_x_async, centroid_y_async, c='green', label='Formation Centre')
        ax_async.grid(True)
        #  # Draw FOV for async agents
        for agent in agents_async:
            fov_direction = math.atan2(target[1] - agent.my_value[1], 
                                    target[0] - agent.my_value[0])
            draw_fov(ax_async, agent.my_value, fov_direction, 
                    math.radians(FOV_ANGLE), FOV_RANGE, 'red', 0.1)
        # Draw FOV for async agents
        # for agent in agents_async:
        #     other_positions = [[a.my_value[0], a.my_value[1]] for a in agents_async if a.id != agent.id]
        #     fov_direction = math.atan2(agent.offset[1], agent.offset[0])
        #     draw_fov(ax_async, agent.my_value, fov_direction, 
        #             math.radians(FOV_ANGLE), FOV_RANGE, 'red', 0.1)
        ax_async.legend()
        update_graph_with_fov(agents, agents_async, target, G, fov_angle=FOV_ANGLE, max_range=FOV_RANGE)
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
        save_frame(fig, count) 

        plt.pause(0.1)


    plt.ioff()
    
    plt.close(fig)
