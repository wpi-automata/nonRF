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
            target_x, target_y = target_position
            ax.scatter(target_x, target_y, c='green', marker='*', s=200, label='Target')
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
            
            if distance < 0.15:  # If close enough to target
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