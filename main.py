from robot import *
from goal import *
from jrrt import *


if __name__ == '__main__':
    # create robot (which also contains simulation environment)
    robot = Robot(with_gui=True)
    for i in range(6):
        goal = Goal(i)
        robot.set_goal(goal)

    robot.set_goal(goal)
    initial_position = robot.ee_position()  # Initial position of the robot
    # Compute joint angles for the goal position
    rrt = J_RRT(robot, goal)  # Initialize J+-RRT planner
    found, path = rrt.plan_path()

    if found:
        print("Path found!")
        # Visualize the path (optional)
        path = np.array(path)
        print(path)
        print(goal._pos)
        """
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(path[:, 0], path[:, 1], path[:, 2], '-o')
        ax.scatter(initial_position[0], initial_position[1],
                   initial_position[2], c='r', marker='o')
        ax.scatter(goal._pos[0], goal._pos[1],
                   goal._pos[2], c='g', marker='o')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('J+-RRT Path Planning')
        plt.show() """
    else:
        print("Path not found!")

    # Simulate the motion (you can add more steps for smoother motion)
    for _ in range(1000):
        p.stepSimulation()

    # Clean up
    robot.disconnect()

"""
    # show all the goals (you would just use one at a time)
    for i in range(6):
        goal = Goal(i)
        robot.set_goal(goal)
        robot = Robot()
        robot.move_to_goal(goal)  # only serves visualization purposes
        robot.disconnect()

    print('---------------------')
    print('the joint limits of the robot are:')
    print('lower: ', robot.joint_limits()[0])
    print('upper: ', robot.joint_limits()[1])

    print('---------------------')
    input('hit enter to continue')

    # some exemplary usage of functions
    joint_pos = [-0.4, 0.4, 0.4, -1.7, 0.0, 1.57, 0.75]
    print('resetting arm joints to:', joint_pos)
    robot.reset_joint_pos(joint_pos)
    goal = Goal(0)
    print('robot is in collision:', robot.in_collision())
    print('end effector position:', robot.ee_position())
    print('EE distance to goal:', goal.distance(robot.ee_position()))
    print('is the goal reached?', goal.reached(robot.ee_position()))
    print('jacobian:', robot.get_jacobian().shape)
    print(robot.get_jacobian())

    print('---------------------')
    input('hit enter to close simulation')
    robot.disconnect()
"""
