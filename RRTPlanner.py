import numpy as np
from RRTTree import RRTTree
import time

class RRTPlanner(object):

    def __init__(self, planning_env, ext_mode, goal_prob):

        # set environment and search tree
        self.planning_env = planning_env
        self.tree = RRTTree(self.planning_env)
        self.step_sizes = [5,10,15]
        # set search params
        self.ext_mode = ext_mode
        self.goal_prob = goal_prob
        self.run_time = None

    def plan(self):
        '''
        Compute and return the plan. The function should return a numpy array containing the states (positions) of the robot.
        '''
        start_time = time.time()

        # initialize an empty plan.
        plan = []
        self.tree.add_vertex(self.planning_env.start)
        testing = []
        new_state = self.planning_env.start
        curr_sampled_position = None
        while self.planning_env.goal[0] != new_state[0] or self.planning_env.goal[1] != new_state[1]:
            sampled_goal = np.random.choice(["Success", "Failure"], p=[self.goal_prob, 1-self.goal_prob])
            if sampled_goal == "Success":
                curr_sampled_position = self.planning_env.goal
            else:
                sampled_x = np.random.randint(self.planning_env.xlimit[0],self.planning_env.xlimit[1])
                sampled_y = np.random.randint(self.planning_env.ylimit[0], self.planning_env.ylimit[1])
                curr_sampled_position = np.array([sampled_x, sampled_y])

            # find nearest neighbour and extend
            nearest_id, nearest_state = self.tree.get_nearest_state(curr_sampled_position)
            new_state = self.extend(nearest_state, curr_sampled_position)
            # check validity
            if not self.planning_env.state_validity_checker(new_state) or not self.planning_env.edge_validity_checker(nearest_state, new_state) or self.tree.get_vertex_for_state(new_state) != None:
                # reset the state to starting state
                new_state = self.planning_env.start
                continue
            new_vertex_id = self.tree.add_vertex(new_state, nearest_id)
            distance = self.planning_env.compute_distance(nearest_state, new_state)
            self.tree.vertices[new_vertex_id].cost = distance
            # print(self.tree.vertices[new_vertex_id].cost)

            self.tree.add_edge(nearest_id, new_vertex_id, distance)
            testing.append(new_state)
            if len(testing) % 1000 == 0:
                print(len(testing))
            
        # find the result path - plan
        curr_state_id = new_vertex_id
        curr_state = new_state
        while self.tree.get_root_id() != curr_state_id:
            plan.append(curr_state)
            curr_vertex = self.tree.get_vertex_for_state(curr_state)
            parent_id = curr_vertex.inspected_points
            parent_vertex = self.tree.vertices[parent_id]
            curr_state = parent_vertex.state
            curr_state_id = curr_vertex.inspected_points
        plan.append(self.planning_env.start)
        plan = plan[::-1]
        # print total path cost and time
        print('Total cost of path: {:.2f}'.format(self.compute_cost(plan)))
        print('Total time: {:.2f}'.format(time.time()-start_time))
        self.run_time = time.time()-start_time

        return np.array(plan)

    def compute_cost(self, plan):
        '''
        Compute and return the plan cost, which is the sum of the distances between steps.
        @param plan A given plan for the robot.
        '''
        return self.tree.get_vertex_for_state(plan[-1]).cost
        
    def extend(self, near_state, rand_state):
        '''
        Compute and return a new position for the sampled one.
        @param near_state The nearest position to the sampled position.
        @param rand_state The sampled position.
        '''
        step_size = self.step_sizes[1]
        if self.ext_mode == "E1":
            return rand_state
        elif self.ext_mode == "E2":
            x1, y1 = near_state
            x2, y2 = rand_state

            # Calculate the direction vector
            dx = x2 - x1
            dy = y2 - y1

            # Compute the Euclidean distance
            distance = self.planning_env.compute_distance(near_state, rand_state)

            # If the distance is less than the step size, we can directly reach the target
            if distance <= step_size:
                return rand_state
            
            # Normalize the direction vector and scale by step_size
            step_x = (dx / distance) * step_size
            step_y = (dy / distance) * step_size

            # Calculate the next position
            next_x = round(x1 + step_x)
            next_y = round(y1 + step_y)

            # Ensure the next position is within the grid bounds
            grid_width, grid_height = self.planning_env.xlimit[1], self.planning_env.ylimit[1]
            next_x = max(0, min(next_x, grid_width))
            next_y = max(0, min(next_y, grid_height))
            return np.array([next_x, next_y])
        else:
            pass