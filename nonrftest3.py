import networkx as nx
from network_agent import DynamicAgent
from distributed_algorithm import run_sim
import matplotlib.pyplot as plt
import random as rand
import time, math

# ---------------------------------------------------
# FormationAgent using asynchronous (non-RF) updates
# ---------------------------------------------------
class FormationAgent(DynamicAgent):
    def __init__(self, id=0, base_value=None, offset=(0, 0)):
        # Initialize the internal consensus (base) value.
        # If no base_value is provided, initialize randomly.
        if base_value is None:
            base_value = [rand.uniform(-10, 10), rand.uniform(-10, 10)]
        self.base_value = base_value  # Internal state used for consensus.
        self.offset = offset  # Fixed positional offset relative to the center.
        
        # The actual (physical) position is computed as base_value + offset.
        self.my_value = [self.base_value[0] + offset[0],
                         self.base_value[1] + offset[1]]
        
        self.id = id
        self.msgs = []  # List to store received messages in the form (sender_id, base_value).

        # For asynchronous updates:
        self.neighbor_sum_x = 0  # Sum of neighbors' x-coordinates.
        self.neighbor_sum_y = 0  # Sum of neighbors' y-coordinates.
        self.lm = 1    # Local multiplier (could be adjusted based on system dynamics).
        self.lo = 5    # Local offset representing the lookout period.
        self.flagcounter = 0  # Controls when updates are performed (asynchronous behavior).

    def msg(self):
        # Send out the current internal consensus value to neighbors.
        return (self.id, self.base_value)

    def stf(self):
        # Increment flagcounter; when it exceeds threshold, perform an update.
        #self.flagcounter += 1
        # We assume that formation is completely synchronous and agents can keep track of that very well

        #if self.flagcounter >= (self.lm + self.lo):
            # Compute rate of change based on neighbors' data.
        rate_x = self.neighbor_sum_x - (len(self.msgs) * self.base_value[0])
        rate_y = self.neighbor_sum_y - (len(self.msgs) * self.base_value[1])

        # Update the internal (base) value using a consensus gain (alpha).
        # This follows the discrete consensus equation from the PDF: Xn+1 = Xn + alpha * (neighbor_sum - N*Xn)
        alpha = 0.3  # Consensus gain factor controlling convergence speed.
        self.base_value[0] = round(self.base_value[0] + alpha * rate_x, 3)
        self.base_value[1] = round(self.base_value[1] + alpha * rate_y, 3)

        # Update the physical position (base + fixed offset).
        self.my_value[0] = round(self.base_value[0] + self.offset[0], 3)
        self.my_value[1] = round(self.base_value[1] + self.offset[1], 3)

        # Reset counters and accumulators for the next cycle.
        #self.flagcounter = 0
        self.clear_msgs()
        self.clear_sum()

    def clear_msgs(self):
        # Clear stored messages after processing.
        self.msgs = []

    def clear_sum(self):
        # Reset neighbor sums to zero.
        self.neighbor_sum_x = 0
        self.neighbor_sum_y = 0

    def add_msg(self, msg):
        # Append a message (sender_id, base_value) and update neighbor sums.
        self.msgs.append(msg)
        self.neighbor_sum_x += msg[1][0]
        self.neighbor_sum_y += msg[1][1]

    def total_msg(self, msg):
        # Not used in this implementation.
        pass

    def step(self):
        # Placeholder for additional logic if needed in future extensions.
        pass

    def setflagcount(self, count):
        # Set the initial flag counter, influencing asynchronous update timing.
        self.flagcounter = count

# ---------------------------------------------------
# CenterAgent that moves toward a target at constant speed
# ---------------------------------------------------
###############################################################
###############################################################
# Steps  which we will be following #####
#1.Agents will approach the target , when they reach near it ,within a specified distance , they make formation
#2.Target will not be considered an agent
#####Phase 1 -All agents can see the target and can transmit the signal to all other agents
#1.Target moves independently. lets say at time t-1 target was at  the position X_t-1 . 
#2.Each Agent will judge the new position of the target (change in position per iteration is velocity in discrete system).At time each agent has 
#its own estimation i.e X_t(i) where i is ith agent
#3.Since agents are transmitting the message through LEDs,few of the agents will judge the position earlier than others, one of the agents will
#start message transmission. There is no bound over who will go first. This way in unbounded situation there is high probability of phase lag as
#LED message transmission will take time and each agent can only send when their time period renews.
#4.Each message transmission of target velocity will be considered as seperate complete consensus. After consensus this position will be saved in 
# place of X_t-1.
#5.This way asynchronous consensus will occur. Once they achieve a final consensus , it will act as velocity to each agent .
#### We can compare these results with synchronous sytem###

#### Benchmarking - 1.When alpha is less than 0.5 , asynchronous system should show a drift
#####               2.When alpha is greater than 0.5 , with adjustment by values from our deduced equation ,asynchronous system will still work 
#####                 but normal system will diverge

######Phase 2 - Only few agents can see the result
######1. Complete connected  graph will not be applicablenanymore
######2.Agents with information will achieve asynchronous consensus
######3.Ideally difference in behaviour of this system depends upon spread of phases. 

#####Phase 3 - Can we apply it on formation control without going into second order systems ?












































class CenterAgent(DynamicAgent):
    def __init__(self, id, value, target):
        self.my_value = value    # Current position [x, y]
        self.target = target     # Desired target position [x, y]
        self.id = id
        self.flagcounter = 0  # Control for asynchronous updates (though not heavily used here).

    def msg(self):
        # Center sends its current position to formation agents.
        return (self.id, self.my_value)

    def stf(self):
        # Move toward the target at a constant speed.
        constant_speed = 0.05  # Speed of movement per iteration.
        dx = self.target[0] - self.my_value[0]
        dy = self.target[1] - self.my_value[1]
        distance = math.sqrt(dx**2 + dy**2)

        if distance < constant_speed:
            # If very close to target, snap to target position.
            self.my_value[0] = self.target[0]
            self.my_value[1] = self.target[1]
        else:
            # Move a fixed step towards the target.
            self.my_value[0] = round(self.my_value[0] + constant_speed * (dx / distance), 3)
            self.my_value[1] = round(self.my_value[1] + constant_speed * (dy / distance), 3)

        self.flagcounter = 0  # Reset flag counter (if using asynchronous logic).

    def clear_msgs(self):
        pass  # Center does not accumulate messages in this implementation.

    def clear_sum(self):
        pass  # Center does not need neighbor sums.

    def add_msg(self, msg):
        pass  # Center does not process neighbor messages.

    def total_msg(self, msg):
        pass

    def step(self):
        pass

    def setflagcount(self, count):
        self.flagcounter = count

# ---------------------------------------------------
# Helper function to create formation agents with specified offsets
# ---------------------------------------------------
def make_formation_agents(n, offsets):
    agents = []
    for i in range(n):
        base_value = [rand.uniform(-10, 10), rand.uniform(-10, 10)]  # Initialize with random base positions.
        agents.append(FormationAgent(i, base_value, offsets[i]))
    return agents

# ---------------------------------------------------
# Main Simulation: Phase 1 (Formation) & Phase 2 (Moving Center)
# ---------------------------------------------------
if __name__ == "__main__":
    # ---------------------------
    # Phase 1: Formation Convergence
    # ---------------------------
    n = 8  # Number of formation agents.

    # Create a graph connecting the formation agents.
    edge_list = [(0, 1), (1, 2), (2, 0), (1, 3),
                 (0, 3), (2, 3), (4, 5), (5, 6),
                 (6, 7), (4, 7), (4, 6), (5, 7)]
    G = nx.Graph()
    G.add_edges_from(edge_list)

    # Define formation offsets (positions relative to the center in a circular formation).
    radius = 5  # Radius of the formation.
    offsets = []
    for i in range(n):
        angle = 2 * math.pi * i / n
        dx = radius * math.cos(angle)
        dy = radius * math.sin(angle)
        offsets.append((dx, dy))

    # Create formation agents.
    formation_agents = make_formation_agents(n, offsets)

    # Set asynchronous update rates for agents (simulate varying transmission periods).
    update_rates = [6, 3, 1, 2, 2, 2, 2, 2]
    for i, rate in enumerate(update_rates):
        formation_agents[i].setflagcount(rate)

    # Create the center agent (starting at [0, 0]).
    center_agent = CenterAgent(n, [0, 0], target=[0, 0])

    # Connect the center to all formation agents.
    for i in range(n):
        G.add_edge(i, n)

    # Combine all agents into one list.
    agents = formation_agents + [center_agent]

    # Set up visualization.
    plt.ion()
    fig, ax = plt.subplots(figsize=(10, 10))
    iterations = 500  # Maximum iterations for phase 1.
    count = 0
    tolerance = 0.1  # Tolerance for convergence.

    #centre agent moves lets assume 3 in x direction and 2 in one direction 

    while count < iterations:
        # Run one simulation cycle.
        agents = run_sim(agents, G, return_agents=True, ctl=True)

        # Check for convergence of formation agents.
        cx, cy = center_agent.my_value
        formation_converged = True
        for agent in formation_agents:
            desired_x = cx + agent.offset[0]
            desired_y = cy + agent.offset[1]
            dx = agent.my_value[0] - desired_x
            dy = agent.my_value[1] - desired_y
            if math.sqrt(dx**2 + dy**2) > tolerance:
                formation_converged = False
                break

        # Visualization: Plot formation agents, center, and desired positions.
        ax.clear()
        ax.set_xlim(-30, 30)
        ax.set_ylim(-30, 30)
        x_vals = [agent.my_value[0] for agent in formation_agents]
        y_vals = [agent.my_value[1] for agent in formation_agents]
        ax.scatter(x_vals, y_vals, c='blue', s=100, label='Formation Agents')
        ax.scatter(cx, cy, c='red', s=150, label='Center')
        desired_xs = [cx + off[0] for off in offsets]
        desired_ys = [cy + off[1] for off in offsets]
        ax.scatter(desired_xs, desired_ys, c='gray', alpha=0.3, s=80, label='Desired Positions')
        ax.legend()
        ax.grid(True)
        plt.pause(0.01)

        if formation_converged:
            print(f"Phase 1 (formation) converged after {count} iterations.")
            break

        count += 1

    # ---------------------------
    # Phase 2: Moving the Center
    # ---------------------------
    # Choose a new random target for the center.
    new_target = [rand.uniform(-20, 20), rand.uniform(-20, 20)]
    center_agent.target = new_target
    print("\nPhase 2: Center target =", new_target)
    count = 0
    iterations_phase2 = 500

    while count < iterations_phase2:
        agents = run_sim(agents, G, return_agents=True, ctl=True)

        # Check if the center and agents have reached the target formation.
        cx, cy = center_agent.my_value
        tx, ty = center_agent.target
        center_error = math.sqrt((cx - tx)**2 + (cy - ty)**2)
        formation_ok = True
        for agent in formation_agents:
            desired_x = cx + agent.offset[0]
            desired_y = cy + agent.offset[1]
            err = math.sqrt((agent.my_value[0] - desired_x)**2 + (agent.my_value[1] - desired_y)**2)
            if err > tolerance:
                formation_ok = False
                break

        # If center and agents are within tolerance, consider it converged.
        if center_error < tolerance and formation_ok:
            print(f"Phase 2 converged after {count} iterations.")
            break

        # Visualization for phase 2.
        ax.clear()
        ax.set_xlim(-30, 30)
        ax.set_ylim(-30, 30)
        x_vals = [agent.my_value[0] for agent in formation_agents]
        y_vals = [agent.my_value[1] for agent in formation_agents]
        ax.scatter(x_vals, y_vals, c='blue', s=100, label='Formation Agents')
        ax.scatter(cx, cy, c='red', s=150, label='Center')
        desired_xs = [cx + agent.offset[0] for agent in formation_agents]
        desired_ys = [cy + agent.offset[1] for agent in formation_agents]
        ax.scatter(desired_xs, desired_ys, c='gray', alpha=0.3, s=80, label='Desired Positions')
        ax.scatter(tx, ty, c='green', s=150, marker='*', label='Target')
        ax.legend()
        ax.grid(True)
        plt.pause(0.01)
        count += 1

    plt.ioff()
    plt.show()

    # Print final positions.
    print("\nFinal positions:")
    print("Center:", center_agent.my_value)
    for agent in formation_agents:
        print(f"Agent {agent.id}: {agent.my_value}")
