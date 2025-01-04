import numpy as np
import heapq

def add_to_queue(item, priority_queue):
    heapq.heappush(priority_queue, item)

def get_lowest_priority_item(priority_queue):
    if priority_queue:
        return heapq.heappop(priority_queue)
    else:
        return None
    
class Node:
    def __init__(self, state, rank ,is_coarse, parent=None):
        self.state = state
        self.rank  = rank
        self.is_coarse = is_coarse
        self.parent = parent
        
    def __lt__(self, other):
        return self.rank > other.rank


class RCSPlanner(object):    
    def __init__(self, planning_env):
        self.planning_env = planning_env

        # used for visualizing the expanded nodes
        # make sure that this structure will contain a list of positions (states, numpy arrays) without duplicates
        self.expanded_nodes = [] 

    def plan(self):
        '''
        Compute and return the plan. The function should return a numpy array containing the states (positions) of the robot.
        '''

        # initialize an empty plan.
        plan = []
        coarse_set = [(2,2), (2,0), (2,-2), (0,-2), (-2,-2), (-2,0), (-2, 2), (0,2)]
        fine_set = [(1,1), (1,0), (1,-1), (0,-1), (-1,-1), (-1,0), (-1,1), (0,1)]
        
        root = Node(self.planning_env.start, 0, is_coarse = False)
        closed = []
        open = []
        add_to_queue(root, open)
        
        while open:
            curr_node = get_lowest_priority_item(open)
            curr_state = curr_node.state
            if self.planning_env.state_validity_checker(curr_state):
                if list(curr_state) not in closed:
                    if curr_state[0] == self.planning_env.goal[0] and curr_state[1] == self.planning_env.goal[1]:
                        plan = self.reconstruct_path(curr_node)
                        return plan
                    for action in coarse_set:
                        new_state = np.array([curr_state[0] + action[0], 
                                              curr_state[1] + action[1]])
                        new_node = Node(new_state, curr_node.rank + 1, is_coarse=True, parent=curr_node)
                        add_to_queue(new_node, open)
                    self.expanded_nodes.append(curr_state)
                    closed.append(list(curr_state))
            # TODO: CHECK if it should be inside valid or outside (pseudo-code is second option)
            if curr_node != root and curr_node.is_coarse:
                parent_state = curr_node.parent.state
                for action in fine_set:
                    new_state = np.array([parent_state[0] + action[0], 
                        parent_state[1] + action[1]])
                    new_node = Node(new_state, curr_node.parent.rank + 1, is_coarse=False, parent=curr_node.parent)
                    # TODO: Check if it should be here:
                    add_to_queue(new_node, open)
    
        return np.array(plan)

    def reconstruct_path(self, node):
        '''
        Reconstruct the path from the goal to the start using parent pointers.
        # YOU DON'T HAVE TO USE THIS FUNCTION!!!
        '''
        path = []
        while node:
            path.append(node.state)  # Append the state
            node = node.parent  # Move to the parent
        path.reverse()
        return np.array(path)
    
    def get_expanded_nodes(self):
        '''
        Return list of expanded nodes without duplicates.
        DO NOT MODIFY THIS FUNCTION!!!
        '''

        # used for visualizing the expanded nodes
        return self.expanded_nodes
