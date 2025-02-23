import networkx as nx
from itertools import product
from network_agent import NetworkAgent
from network_agent import DynamicAgent


def run_sim(agents: list[DynamicAgent], G: nx.graph, return_agents: bool = False, ctl: bool = False):
    '''
    This function will execute a generic distributed algorithm as discussed in RBE 510.
    It operates on the NetworkAgent subclass and its related methods.

    Parameters
    ----------
    agents: list of NetworkAgent or a subclass thereof
    G: a networkx graph representing the communication topology of the agents
    max_iter: an integer specifying the maximum number of iterations to simulate
    return_agents: a Boolean specifying whether or not to output the list of agents
    ctl: a Boolean specifying whether the physical state of the agents should be simulated

    Returns
    -------
    agents: list of input agents with in-place updates (if return_agents == True)
    '''


    n = len(agents)
    

    # Main simulation loop
        
       


        # Send messages for each edge in the graph
    for ii,jj in product(range(n),range(n)):
        if (ii,jj) in G.edges and ii!=jj:
            agents[ii].add_msg(agents[jj].msg())
            agents[ii].total_msg(agents[jj].msg())
            
                
        # Execute state transition function for each agent
    for ii in range(n):
        agents[ii].stf()
        
        # Simulate dynamics if they are included
    if ctl == True:

            # Compute control action
       # for ii in range(n):
            #agents[ii].ctl()

            # Step agents forward
        for ii in range(n):
            agents[ii].step()

        # Clear messages for next round
    for ii in range(n):
            agents[ii].clear_msgs()
            agents[ii].clear_sum()
    
    # Return output
    if return_agents:
        return agents
    