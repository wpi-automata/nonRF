import networkx as nx
from network_agent import DynamicAgent
from distributed_algorithm import run_sim
import matplotlib.pyplot as plt
import random as rand
import time, math

# Global list to store in-flight (animated) messages.
inflight_messages = []  # Each message is a dict with keys: source, target, start_pos, end_pos, start_time, duration, color

# ---------------------------------------------------
# Helper: Retrieve an agent by id from a list of agents.
# ---------------------------------------------------
def get_agent_by_id(agents, agent_id):
    for a in agents:
        if a.id == agent_id:
            return a
    return None

# ---------------------------------------------------
# FormationAgent using asynchronous (non-RF) updates
# ---------------------------------------------------
class FormationAgent(DynamicAgent):
    def __init__(self, id=0, base_value=None, offset=(0, 0)):
        # Initialize the internal consensus (base) value.
        if base_value is None:
            base_value = [rand.uniform(-10, 10), rand.uniform(-10, 10)]
        self.base_value = base_value  # Internal state used for consensus.
        self.offset = offset  # Fixed positional offset relative to the center.
        
        # The actual (physical) position is computed as base_value + offset.
        self.my_value = [self.base_value[0] + offset[0],
                         self.base_value[1] + offset[1]]
        
        self.id = id
        self.msgs = []  # List to store received messages (sender_id, base_value).

        # For asynchronous updates:
        self.neighbor_sum_x = 0  # Sum of neighbors' x-values.
        self.neighbor_sum_y = 0  # Sum of neighbors' y-values.
        self.lm = 1    # Local multiplier (adjustable based on dynamics).
        self.lo = 5    # Local offset representing the lookout period.
        self.flagcounter = 0  # Controls when updates are performed.
        self.just_updated = False  # Flag indicating that the agent has just updated.

    def msg(self):
        # Send out the current internal consensus value to neighbors.
        return (self.id, self.base_value)

    def stf(self):
        # Increment flagcounter; when it exceeds threshold, perform an update.
        self.flagcounter += 1
        if self.flagcounter >= (self.lm + self.lo):
            self.just_updated = True
            # Compute rate of change based on neighbor messages.
            rate_x = self.neighbor_sum_x - (len(self.msgs) * self.base_value[0])
            rate_y = self.neighbor_sum_y - (len(self.msgs) * self.base_value[1])
            alpha = 0.2  # Reduced consensus gain factor (was 0.3)
            self.base_value[0] = round(self.base_value[0] + alpha * rate_x, 3)
            self.base_value[1] = round(self.base_value[1] + alpha * rate_y, 3)

            # Update physical position (base + offset).
            self.my_value[0] = round(self.base_value[0] + self.offset[0], 3)
            self.my_value[1] = round(self.base_value[1] + self.offset[1], 3)

            # Reset counters and accumulators.
            self.flagcounter = 0
            self.clear_msgs()
            self.clear_sum()
        else:
            self.just_updated = False

    def clear_msgs(self):
        self.msgs = []

    def clear_sum(self):
        self.neighbor_sum_x = 0
        self.neighbor_sum_y = 0

    def add_msg(self, msg):
        self.msgs.append(msg)
        self.neighbor_sum_x += msg[1][0]
        self.neighbor_sum_y += msg[1][1]

    def total_msg(self, msg):
        pass

    def step(self):
        pass

    def setflagcount(self, count):
        self.flagcounter = count

# ---------------------------------------------------
# CenterAgent that moves toward a target at constant speed
# ---------------------------------------------------
class CenterAgent(DynamicAgent):
    def __init__(self, id, value, target):
        self.my_value = value    # Current position [x, y]
        self.target = target     # Desired target position [x, y]
        self.id = id
        self.flagcounter = 0

    def msg(self):
        return (self.id, self.my_value)

    def stf(self):
        constant_speed = 0.05  # Speed per iteration.
        dx = self.target[0] - self.my_value[0]
        dy = self.target[1] - self.my_value[1]
        distance = math.sqrt(dx**2 + dy**2)
        if distance < constant_speed:
            self.my_value[0] = self.target[0]
            self.my_value[1] = self.target[1]
        else:
            self.my_value[0] = round(self.my_value[0] + constant_speed * (dx / distance), 3)
            self.my_value[1] = round(self.my_value[1] + constant_speed * (dy / distance), 3)
        self.flagcounter = 0

    def clear_msgs(self):
        pass

    def clear_sum(self):
        pass

    def add_msg(self, msg):
        pass

    def total_msg(self, msg):
        pass

    def step(self):
        pass

    def setflagcount(self, count):
        self.flagcounter = count

# ---------------------------------------------------
# Helper function to create formation agents with specified offsets.
# ---------------------------------------------------
def make_formation_agents(n, offsets):
    agents = []
    for i in range(n):
        base_value = [rand.uniform(-10, 10), rand.uniform(-10, 10)]
        agents.append(FormationAgent(i, base_value, offsets[i]))
    return agents

# ---------------------------------------------------
# Main Simulation: Phase 1 (Formation) & Phase 2 (Moving Center)
# ---------------------------------------------------
if __name__ == "__main__":
    n = 8  # Number of formation agents.
    
    # Create a graph and add complete connections among formation agents.
    G = nx.Graph()
    # Add complete edges among formation agents (nodes 0 to n-1)
    formation_nodes = list(range(n))
    for i in formation_nodes:
        for j in formation_nodes:
            if i < j:
                G.add_edge(i, j)
    
    # Create the center node with id n and add edges connecting it to every formation agent.
    center_id = n
    for i in formation_nodes:
        G.add_edge(i, center_id)

    # Define formation offsets (positions relative to the center in a circular formation).
    radius = 5
    offsets = []
    for i in range(n):
        angle = 2 * math.pi * i / n
        dx = radius * math.cos(angle)
        dy = radius * math.sin(angle)
        offsets.append((dx, dy))

    # Create formation agents.
    formation_agents = make_formation_agents(n, offsets)
    # Set asynchronous update rates for agents (simulating varying transmit/lookout periods).
    update_rates = [6, 3, 1, 2, 2, 2, 2, 2]
    for i, rate in enumerate(update_rates):
        formation_agents[i].setflagcount(rate)

    # Create the center agent (with id = n) starting at (0, 0).
    center_agent = CenterAgent(center_id, [0, 0], target=[0, 0])

    # Combine all agents into one list.
    agents = formation_agents + [center_agent]

    # Create a color mapping for messages from each formation agent.
    msg_colors = {}
    colormap = plt.cm.get_cmap('tab10')
    for agent in formation_agents:
        msg_colors[agent.id] = colormap(agent.id % 10)

    # Set up visualization.
    plt.ion()
    fig, ax = plt.subplots(figsize=(10, 10))
    iterations = 500  # Maximum iterations for phase 1.
    count = 0
    tolerance = 0.1  # Tolerance for convergence.

    # Simulation loop for Phase 1.
    while count < iterations:
        agents = run_sim(agents, G, return_agents=True, ctl=True)
        current_time = time.time()

        # For each formation agent that just updated, create in-flight message events.
        for agent in formation_agents:
            if agent.just_updated:
                for neighbor_id in G.neighbors(agent.id):
                    neighbor_agent = get_agent_by_id(agents, neighbor_id)
                    if neighbor_agent is not None:
                        message = {
                            'source': agent.id,
                            'target': neighbor_agent.id,
                            'start_pos': agent.my_value.copy(),
                            'end_pos': neighbor_agent.my_value.copy(),
                            'start_time': current_time,
                            'duration': 1.0,  # Message transit duration (in seconds).
                            'color': msg_colors[agent.id]
                        }
                        inflight_messages.append(message)
                agent.just_updated = False

        # Visualization.
        ax.clear()
        ax.set_xlim(-30, 30)
        ax.set_ylim(-30, 30)

        # Draw formation agents.
        x_vals = [agent.my_value[0] for agent in formation_agents]
        y_vals = [agent.my_value[1] for agent in formation_agents]
        ax.scatter(x_vals, y_vals, c='blue', s=100, label='Formation Agents')

        # Draw the center agent.
        cx, cy = center_agent.my_value
        ax.scatter(cx, cy, c='red', s=150, label='Center')

        # Draw desired positions (center + offset).
        desired_xs = [cx + off[0] for off in offsets]
        desired_ys = [cy + off[1] for off in offsets]
        ax.scatter(desired_xs, desired_ys, c='gray', alpha=0.3, s=80, label='Desired Positions')

        # Animate in-flight messages.
        for message in inflight_messages[:]:
            progress = (current_time - message['start_time']) / message['duration']
            if progress >= 1:
                inflight_messages.remove(message)
            else:
                sx, sy = message['start_pos']
                ex, ey = message['end_pos']
                msg_x = sx + (ex - sx) * progress
                msg_y = sy + (ey - sy) * progress
                ax.plot([sx, msg_x], [sy, msg_y], linestyle='--', linewidth=1, color=message['color'])
                ax.scatter(msg_x, msg_y, color=message['color'], s=20)

        ax.legend()
        ax.grid(True)
        plt.pause(0.05)
        count += 1

        # Check convergence: formation agents should be near (center + offset).
        formation_converged = True
        for agent in formation_agents:
            desired_x = cx + agent.offset[0]
            desired_y = cy + agent.offset[1]
            dx = agent.my_value[0] - desired_x
            dy = agent.my_value[1] - desired_y
            if math.sqrt(dx**2 + dy**2) > tolerance:
                formation_converged = False
                break
        if formation_converged:
            print(f"Phase 1 (formation) converged after {count} iterations.")
            break

    # ---------------------------
    # Phase 2: Moving the Center (and keeping formation)
    # ---------------------------
    new_target = [rand.uniform(-20, 20), rand.uniform(-20, 20)]
    center_agent.target = new_target
    print("\nPhase 2: Center target =", new_target)
    count = 0
    iterations_phase2 = 500

    while count < iterations_phase2:
        agents = run_sim(agents, G, return_agents=True, ctl=True)
        current_time = time.time()

        for agent in formation_agents:
            if agent.just_updated:
                for neighbor_id in G.neighbors(agent.id):
                    neighbor_agent = get_agent_by_id(agents, neighbor_id)
                    if neighbor_agent is not None:
                        message = {
                            'source': agent.id,
                            'target': neighbor_agent.id,
                            'start_pos': agent.my_value.copy(),
                            'end_pos': neighbor_agent.my_value.copy(),
                            'start_time': current_time,
                            'duration': 1.0,
                            'color': msg_colors[agent.id]
                        }
                        inflight_messages.append(message)
                agent.just_updated = False

        ax.clear()
        ax.set_xlim(-30, 30)
        ax.set_ylim(-30, 30)
        x_vals = [agent.my_value[0] for agent in formation_agents]
        y_vals = [agent.my_value[1] for agent in formation_agents]
        ax.scatter(x_vals, y_vals, c='blue', s=100, label='Formation Agents')
        cx, cy = center_agent.my_value
        ax.scatter(cx, cy, c='red', s=150, label='Center')
        desired_xs = [cx + agent.offset[0] for agent in formation_agents]
        desired_ys = [cy + agent.offset[1] for agent in formation_agents]
        ax.scatter(desired_xs, desired_ys, c='gray', alpha=0.3, s=80, label='Desired Positions')
        tx, ty = center_agent.target
        ax.scatter(tx, ty, c='green', s=150, marker='*', label='Target')

        for message in inflight_messages[:]:
            progress = (current_time - message['start_time']) / message['duration']
            if progress >= 1:
                inflight_messages.remove(message)
            else:
                sx, sy = message['start_pos']
                ex, ey = message['end_pos']
                msg_x = sx + (ex - sx) * progress
                msg_y = sy + (ey - sy) * progress
                ax.plot([sx, msg_x], [sy, msg_y], linestyle='--', linewidth=1, color=message['color'])
                ax.scatter(msg_x, msg_y, color=message['color'], s=20)

        ax.legend()
        ax.grid(True)
        plt.pause(0.05)
        count += 1

        center_error = math.sqrt((cx - tx)**2 + (cy - ty)**2)
        formation_ok = True
        for agent in formation_agents:
            desired_x = cx + agent.offset[0]
            desired_y = cy + agent.offset[1]
            err = math.sqrt((agent.my_value[0] - desired_x)**2 + (agent.my_value[1] - desired_y)**2)
            if err > tolerance:
                formation_ok = False
                break
        if center_error < tolerance and formation_ok:
            print(f"Phase 2 converged after {count} iterations.")
            break

    plt.ioff()
    plt.show()

    # Print final positions.
    print("\nFinal positions:")
    print("Center:", center_agent.my_value)
    for agent in formation_agents:
        print(f"Agent {agent.id}: {agent.my_value}")
