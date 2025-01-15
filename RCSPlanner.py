import numpy as np

class Node:
    def __init__(self, state, rank ,is_coarse, parent=None):
        self.state = state
        self.rank  = rank
        self.is_coarse = is_coarse
        self.parent = parent
        
    def __lt__(self, other):
        return self.rank > other.rank
    
    def __eq__(self, other):
        return isinstance(other, Node) and tuple(self.state) == tuple(other.state)

    def __hash__(self):
        return hash(tuple(self.state))

class RCSPlanner(object):    
    def __init__(self, planning_env):
        self.planning_env = planning_env

        # used for visualizing the expanded nodes
        # make sure that this structure will contain a list of positions (states, numpy arrays) without duplicates
        self.expanded_nodes = [] 


    # def plan(self):
    #     '''
    #     Compute and return the plan. The function should return a numpy array containing the states (positions) of the robot.
    #     '''

    #     # initialize an empty plan.
    #     plan = []
    #     # coarse_set = [(2,2), (2,0), (2,-2), (0,-2), (-2,-2), (-2,0), (-2, 2), (0, 2)]
    #     # fine_set = [(1,1), (1,0), (1,-1), (0,-1), (-1,-1), (-1,0), (-1,1), (0,1)]

    #     coarse_set = [(-2,-2), (-2,0), (-2,2), (0,-2), (0,2), (2,-2), (2, 0), (2,2)]
    #     fine_set = [(-1,-1), (-1,0), (-1,1), (0,-1), (0,1), (1,-1), (1,0), (1,1)]
    #     # coarse_set = {(2,2), (2,0), (2,-2), (0,2), (0,-2), (-2,2), (-2,0), (-2,-2)}
    #     # fine_set = {(1,1), (1,0), (1,-1), (0,1), (0,-1), (-1,1), (-1,0), (-1,-1)}
    #     root = Node(self.planning_env.start, 0, is_coarse = True)
    #     closed = []
    #     open = []
    #     open.append((root, root.rank))
    #     while open:
    #         curr_node, _ =  min(open, key=lambda x: x[1])
    #         curr_state = tuple(curr_node.state)
    #         open.remove((curr_node, curr_node.rank))
    #         if curr_state[0] == self.planning_env.goal[0] and curr_state[1] == self.planning_env.goal[1]:
    #             plan = self.reconstruct_path(curr_node)
    #             return plan
    #         if not self.planning_env.state_validity_checker(curr_state):
    #             continue
    #         if curr_node in closed:
    #             continue
    #         for action in coarse_set:
    #             new_state = np.array([curr_state[0] + action[0],
    #                                     curr_state[1] + action[1]])
    #             if self.planning_env.edge_validity_checker(curr_state, new_state):
    #                 new_node = Node(new_state, curr_node.rank + 1, is_coarse=True, parent=curr_node)
    #                 open.append((new_node, curr_node.rank + 1))
    #             # else:
    #                 # print(f"Can't be edge between coarse:{curr_state[0]},{curr_state[1]} to {new_state[0]},{new_state[1]}")

    #         self.expanded_nodes.append(curr_state)
    #         closed.append(curr_node)

    #         if curr_state != tuple(self.planning_env.start) and curr_node.is_coarse:
    #             parent_state = curr_node.parent.state
    #             for action in fine_set:
    #                 new_state = np.array([parent_state[0] + action[0],
    #                     parent_state[1] + action[1]])
    #                 if self.planning_env.edge_validity_checker(curr_state, new_state):
    #                     new_node = Node(new_state, curr_node.rank + 1, is_coarse=False, parent=curr_node.parent)
    #                     open.append((new_node, curr_node.rank + 1))
    #                 # else:
    #                     # print(f"Can't be edge between fine:{curr_state[0]},{curr_state[1]} to {new_state[0]},{new_state[1]}")

    #     return np.array(plan)

    def plan(self):
        '''
        Compute and return the plan. The function should return a numpy array containing the states (positions) of the robot.
        '''
        # Initialize plan, open list, and closed set
        plan = []
        open = []
        closed = set()

        # coarse_set = [(2,2), (2,0), (2,-2), (0,-2), (-2,-2), (-2,0), (-2, 2), (0, 2)]
        # fine_set = [(1,1), (1,0), (1,-1), (0,-1), (-1,-1), (-1,0), (-1,1), (0,1)]

        # Define coarse and fine movement sets
        coarse_set = [(-2, -2), (-2, 0), (-2, 2), (0, -2), (0, 2), (2, -2), (2, 0), (2, 2)]
        fine_set = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]

        # Initialize the root node
        root = Node(self.planning_env.start, rank=0, is_coarse=True)
        open.append((root, root.rank))

        while open:
            # Get the node with the smallest rank
            curr_node, _ = min(open, key=lambda x: x[1])
            open.remove((curr_node, curr_node.rank))
            curr_state = tuple(curr_node.state)

            # If the goal is reached, reconstruct and return the path
            if curr_state == tuple(self.planning_env.goal):
                plan = self.reconstruct_path(curr_node)
                return np.array(plan)

            # Skip invalid or already visited states
            if not self.planning_env.state_validity_checker(curr_state) or curr_node in closed:
                continue

            # Mark current node as visited
            closed.add(curr_node)

            # Expand coarse neighbors
            for action in coarse_set:
                new_state = np.array([curr_state[0] + action[0], curr_state[1] + action[1]])
                if self.planning_env.edge_validity_checker(curr_state, new_state):
                    new_node = Node(new_state, curr_node.rank + 1, is_coarse=True, parent=curr_node)
                    open.append((new_node, new_node.rank))
                # else:
                    # print(f"Not valid edges:{(curr_state[0],curr_state[1])}->{(new_state[0],new_state[1])}")

            # Expand fine neighbors if not the root and the current node is coarse
            if curr_state != tuple(self.planning_env.start) and curr_node.is_coarse:
                parent_state = curr_node.parent.state
                for action in fine_set:
                    new_state = np.array([parent_state[0] + action[0], parent_state[1] + action[1]])
                    if self.planning_env.edge_validity_checker(curr_state, new_state):
                        new_node = Node(new_state, curr_node.rank + 1, is_coarse=False, parent=curr_node.parent)
                        open.append((new_node, new_node.rank))
            # Keep track of expanded nodes for debugging or analysis
            self.expanded_nodes.append(curr_state)

        # Return the computed plan
        return np.array(plan)

    def reconstruct_path(self, node):
        '''
        Reconstruct the path from the goal to the start using parent pointers.
        # YOU DON'T HAVE TO USE THIS FUNCTION!!!
        '''
        fine_steps,coarse_steps = 0,0
        total_distance = 0
        path = []
        prev_node = node
        while node:
            total_distance += np.linalg.norm(prev_node.state - node.state)

            if not np.array_equal(node.state, self.planning_env.start):
                fine_steps += not node.is_coarse
                coarse_steps += node.is_coarse

            path.append(node.state)  # Append the state
            prev_node = node
            node = node.parent  # Move to the parent
        path.reverse()

        total_steps = coarse_steps + fine_steps
        print(f"Total Steps: {total_steps}.")
        print(f"Coarse Steps: {coarse_steps}, {coarse_steps / total_steps * 100:.2f}% of total steps.")
        print(f"Fine Steps: {fine_steps}, {fine_steps / total_steps * 100:.2f}% of total steps.")
        print(f"Total Distance:{total_distance:.2f} units.")
        return np.array(path)

    def get_expanded_nodes(self):
        '''
        Return list of expanded nodes without duplicates.
        DO NOT MODIFY THIS FUNCTION!!!
        '''

        # used for visualizing the expanded nodes
        return self.expanded_nodes
