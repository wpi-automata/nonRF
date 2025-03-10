import math
import os
import matplotlib.pyplot as plt
import networkx as nx
import numpy as np

def update_agent_fov_directions(agents, target, neighbors_only=False):
    """Updates FOV directions for all agents consistently"""
    for agent in agents:
        if neighbors_only:
            # Formation phase - face formation neighbors
            fov_direction = math.atan2(agent.offset[1], agent.offset[0])
        else:
            # Tracking phase - balance between target and neighbors
            target_direction = math.atan2(target[1] - agent.my_value[1], 
                                        target[0] - agent.my_value[0])
            neighbor_direction = math.atan2(agent.offset[1], agent.offset[0])
            # Weighted combination (70% target, 30% formation)
            fov_direction = 0.7 * target_direction + 0.3 * neighbor_direction
            
        agent.fov_direction = fov_direction
        
def calculate_fov_direction(agent, target_pos, neighbor_positions):
    """Dynamic FOV direction balancing target and neighbor visibility"""
    target_vector = np.array(target_pos) - np.array(agent.my_value)
    neighbor_vector = np.array([0.0, 0.0])
    
    if neighbor_positions:
        # Find closest neighbor
        distances = [np.linalg.norm(np.array(pos) - agent.my_value) 
                    for pos in neighbor_positions]
        closest_idx = np.argmin(distances)
        neighbor_vector = np.array(neighbor_positions[closest_idx]) - agent.my_value
    
    # Weighted combination (60% target, 40% neighbor)
    combined_vector = 0.6 * target_vector + 0.4 * neighbor_vector
    
    if np.linalg.norm(combined_vector) < 0.1:  # Fallback if no clear direction
        return agent.fov_direction  # Maintain current direction
    
    return np.arctan2(combined_vector[1], combined_vector[0])

def modified_fov_adaptation(agent_pos, other_positions, target, current_direction, fov_angle, max_range):
    """
    Adjusts the agent's FOV direction based on visible neighbors and the target.
    """
    # Count currently visible neighbors
    visible_neighbors = sum(1 for pos in other_positions if 
                          check_in_fov(agent_pos, pos, current_direction, math.radians(fov_angle), max_range))
    
    # If at least one neighbor is visible, maintain current direction with slight bias toward target
    if visible_neighbors >= 1:
        target_direction = math.atan2(target[1] - agent_pos[1], target[0] - agent_pos[0])
        # Small adjustment toward target (10% target influence)
        adjusted_direction = current_direction * 0.9 + target_direction * 0.1
        return adjusted_direction
    
    # If no neighbors visible, scan for neighbors
    closest_neighbor_idx = -1
    closest_distance = float('inf')
    
    for i, pos in enumerate(other_positions):
        distance = math.sqrt((pos[0] - agent_pos[0])**2 + (pos[1] - agent_pos[1])**2)
        if distance < closest_distance and distance <= max_range:
            closest_distance = distance
            closest_neighbor_idx = i
    
    # If found neighbor within range, point toward it
    if closest_neighbor_idx != -1:
        neighbor_pos = other_positions[closest_neighbor_idx]
        return math.atan2(neighbor_pos[1] - agent_pos[1], neighbor_pos[0] - agent_pos[0])
    
    # Otherwise point toward target
    return math.atan2(target[1] - agent_pos[1], target[0] - agent_pos[0])

def check_minimum_connectivity(agents, G):
    """
    Identifies agents that are disconnected in the graph.
    """
    disconnected_agents = []
    
    for agent in agents:
        if G.degree(agent.id) == 0:
            disconnected_agents.append(agent.id)
            
    return disconnected_agents

def recover_isolated_agents(agents, disconnected_ids, target, formation_weight=0.6, target_weight=0.4, recovery_step=5.0):
    """
    Moves disconnected agents toward the centroid of connected agents and the target.
    """
    # Get centroid of connected agents
    connected_agents = [a for a in agents if a.id not in disconnected_ids]
    
    if not connected_agents:
        # All agents disconnected, move all toward target
        for agent_id in disconnected_ids:
            agent = agents[agent_id]
            target_vector = [target[0] - agent.my_value[0], target[1] - agent.my_value[1]]
            target_dist = math.sqrt(target_vector[0]**2 + target_vector[1]**2)
            
            if target_dist > 0:
                direction = [v/target_dist for v in target_vector]
                agent.my_value[0] += direction[0] * recovery_step
                agent.my_value[1] += direction[1] * recovery_step
        return
    
    centroid = [sum(a.my_value[0] for a in connected_agents)/len(connected_agents),
               sum(a.my_value[1] for a in connected_agents)/len(connected_agents)]
    
    for agent_id in disconnected_ids:
        agent = agents[agent_id]
        
        # Move partially toward target and partially toward formation centroid
        formation_vector = [centroid[0] - agent.my_value[0], centroid[1] - agent.my_value[1]]
        target_vector = [target[0] - agent.my_value[0], target[1] - agent.my_value[1]]
        
        # Normalize and weight vectors
        formation_dist = math.sqrt(formation_vector[0]**2 + formation_vector[1]**2)
        target_dist = math.sqrt(target_vector[0]**2 + target_vector[1]**2)
        
        if formation_dist > 0:
            formation_vector = [formation_weight * v/formation_dist for v in formation_vector]
        
        if target_dist > 0:
            target_vector = [target_weight * v/target_dist for v in target_vector]
        
        # Apply small movement in combined direction
        combined_vector = [formation_vector[0] + target_vector[0], formation_vector[1] + target_vector[1]]
        combined_dist = math.sqrt(combined_vector[0]**2 + combined_vector[1]**2)
        
        if combined_dist > 0:
            direction = [v/combined_dist for v in combined_vector]
            agent.my_value[0] += direction[0] * recovery_step
            agent.my_value[1] += direction[1] * recovery_step

def check_in_fov(agent1_pos, agent2_pos, fov_direction, fov_angle, max_range):
    """
    Check if agent2 is within agent1's field of view.
    """
    # Calculate distance
    dx = agent2_pos[0] - agent1_pos[0]
    dy = agent2_pos[1] - agent1_pos[1]
    distance = math.sqrt(dx**2 + dy**2)
    
    if distance > max_range:
        return False
        
    # Calculate angle to target
    angle = math.atan2(dy, dx)
    
    # Calculate angular difference
    angle_diff = abs((angle - fov_direction + math.pi) % (2*math.pi) - math.pi)
    
    return angle_diff <= fov_angle/2

def update_graph_with_fov(agents, agents_async, target_pos, G, fov_angle, max_range):
    """
    Updates graph edges based on FOV constraints.
    """
    # Convert FOV angle to radians
    fov_rad = math.radians(fov_angle)
    
    # Clear existing edges
    G.clear_edges()
    
    # For synchronous agents
    for i, agent1 in enumerate(agents):
        target_direction = math.atan2(target_pos[1] - agent1.my_value[1], 
                                    target_pos[0] - agent1.my_value[0])
        
        for j, agent2 in enumerate(agents):
            if i != j:
                if (check_in_fov(agent1.my_value, agent2.my_value, target_direction, fov_rad, max_range) and
                    check_in_fov(agent2.my_value, agent1.my_value, 
                               math.atan2(target_pos[1] - agent2.my_value[1], 
                                        target_pos[0] - agent2.my_value[0]), 
                               fov_rad, max_range)):
                    G.add_edge(i, j)
                    # Add this debug statement:
                    print(f"Agent {i} sees {[j for j in range(len(agents)) if G.has_edge(i,j)]}")
    
    # For asynchronous agents
    for i, agent1 in enumerate(agents_async):
        target_direction = math.atan2(target_pos[1] - agent1.my_value[1], 
                                    target_pos[0] - agent1.my_value[0])
        
        for j, agent2 in enumerate(agents_async):
            if i != j:
                if (check_in_fov(agent1.my_value, agent2.my_value, target_direction, fov_rad, max_range) and
                    check_in_fov(agent2.my_value, agent1.my_value,
                               math.atan2(target_pos[1] - agent2.my_value[1], 
                                        target_pos[0] - agent2.my_value[0]),
                               fov_rad, max_range)):
                    G.add_edge(i+len(agents), j+len(agents))
                    # Add this debug statement:
                    print(f"Agent {i} sees {[j for j in range(len(agents)) if G.has_edge(i,j)]}")
    
    # Print graph information for debugging
    print(f"Number of edges in graph: {G.number_of_edges()}")
    print(f"Graph connectivity: {nx.is_connected(G)}")

def draw_fov(ax, agent_pos, fov_direction, fov_angle, max_range, color='gray', alpha=0.1):
    """
    Draw the FOV of an agent as a circular sector.
    """
    # Convert FOV angle to degrees for matplotlib
    fov_deg = math.degrees(fov_angle)
    start_angle = math.degrees(fov_direction - fov_angle/2)
    
    # Create circular sector
    wedge = plt.matplotlib.patches.Wedge(
        (agent_pos[0], agent_pos[1]), 
        max_range, 
        start_angle, 
        start_angle + fov_deg, 
        color=color, 
        alpha=alpha
    )
    ax.add_patch(wedge)

def save_frame(fig, frame_num, output_dir='frames'):
    """
    Save the current plot as an image frame.
    """
    # Create output directory if it doesn't exist
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    # Save the figure
    fig.savefig(f'{output_dir}/frame_{frame_num:05d}.png')