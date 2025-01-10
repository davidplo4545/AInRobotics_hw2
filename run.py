import argparse
from MapEnvironment import MapEnvironment
from RRTPlanner import RRTPlanner
from RRTStarPlanner import RRTStarPlanner
from RCSPlanner import RCSPlanner

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description='script for testing planners')
    parser.add_argument('-map', '--map', type=str, default='map1.json', help='Json file name containing all map information')
    parser.add_argument('-planner', '--planner', type=str, default='rrt', help='The planner to run. Choose from [astar, rrt, rrtstar]')
    parser.add_argument('-ext_mode', '--ext_mode', type=str, default='E1', help='edge extension mode for RRT and RRTStar')
    parser.add_argument('-goal_prob', '--goal_prob', type=float, default=0.05, help='probability to draw goal vertex for RRT and RRTStar')
    parser.add_argument('-k', '--k', type=int, default=1, help='number of nearest neighbours for RRTStar')
    args = parser.parse_args()

    # prepare the map
    planning_env = MapEnvironment(json_file=args.map)

    # setup the planner
    if args.planner == 'rcs':
        planner = RCSPlanner(planning_env=planning_env)
    elif args.planner == 'rrt':
        planner = RRTPlanner(planning_env=planning_env, ext_mode=args.ext_mode, goal_prob=args.goal_prob)
    elif args.planner == 'rrtstar':
        planner = RRTStarPlanner(planning_env=planning_env, ext_mode=args.ext_mode, goal_prob=args.goal_prob, k=args.k)
    else:
        raise ValueError('Unknown planner option: %s' % args.planner);



    # path_costs = []
    # run_times = []
    # for i in range(10):
    #     planning_env = MapEnvironment(json_file=args.map)
    #     planner = RRTPlanner(planning_env=planning_env, ext_mode=args.ext_mode, goal_prob=args.goal_prob)
    #     plan = planner.plan()
    #     path_cost = planner.compute_cost(plan)
    #     run_time = planner.run_time
    #     path_costs.append(path_cost)
    #     run_times.append(run_time)
    #     print(f"Done iteration: {i}")
    #     break
    #     # Generate x-axis values: index + 1
    # indices = range(1, 11)

    # import matplotlib.pyplot as plt
    # # Plot the data
    # plt.plot(indices, path_costs, marker='o', linestyle='-')
    # plt.xlabel('Run No.')
    # plt.ylabel('Path Cost')
    # plt.title(f'Path Cost For:{args.ext_mode} with Goal Probability:{args.goal_prob}')
    # avg_path_costs = sum(path_costs) / float(len(path_costs))
    # print(f"Average Path Costs is:{avg_path_costs}")
    # plt.axhline(y=avg_path_costs, color='r', linestyle='--')
    # plt.grid(True)
    # plt.show()

    # plt.plot(indices, run_times, marker='o', linestyle='-')
    # plt.xlabel('Run No.')
    # plt.ylabel('Run Time')
    # plt.title(f'Runtime For:{args.ext_mode} with Goal Probability:{args.goal_prob}')
    # avg_runtime = sum(run_times) / float(len(run_times))
    # plt.axhline(y=avg_runtime, color='r', linestyle='--')
    # print(f"Average run time is:{avg_runtime}")
    # plt.grid(True)
    # plt.show()

    # execute plan
    plan = planner.plan()

    # visualize the final path with edges or states according to the requested planner.
    if args.planner == 'rcs':
        planner.planning_env.visualize_map(plan=plan, expanded_nodes=planner.get_expanded_nodes())
    else:
        planner.planning_env.visualize_map(plan=plan, tree_edges=planner.tree.get_edges_as_states())