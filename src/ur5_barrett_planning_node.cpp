/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta */

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <signal.h>

// Check if value is in given range
bool inRange(float min, float max, float value)
{
	return (value < max) && (value > min);
}

// Check if pose is inside the workspace
bool isPoseFeasible(float x, float y, float z)
{
	if (inRange(-1.0, 1.0, x) && inRange(0.0, 1.5, y) && inRange(0.0, 0.7, z))
		return true;
	else
		return false;
}

void exitGracefully(int sig)
{
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ur5_barrett_planning_node");
	ros::NodeHandle node_handle;
	signal(SIGINT, exitGracefully);
	ros::AsyncSpinner spinner(1);
	spinner.start();


	/* This sleep is ONLY to allow Rviz to come up */
	sleep(20.0);

	// BEGIN_TUTORIAL
	// 
	// Setup
	// ^^^^^
	// 
	// The :move_group_interface:`MoveGroup` class can be easily 
	// setup using just the name
	// of the group you would like to control and plan for.
	moveit::planning_interface::MoveGroup group("arm");

	// We will use the :planning_scene_interface:`PlanningSceneInterface`
	// class to deal directly with the world.
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

	// Create a publisher for visualizing plans in Rviz.
	ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

	// Create a publisher to visulize the goal pose
	ros::Publisher display_goal_pose_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("/move_group/display_goal_pose", 1);

	// Specify a planner to be used for further planning.
	group.setPlannerId("RRTConnectkConfigDefault");

	// Set the tolerance that is used for reaching the goal
	group.setGoalTolerance(0.025);

	// Set workspace
	// note: planning and execution doesn't take this into account.
	group.setWorkspace(-1.0, -0.35, 0.0, 1.0, 0.35, 1.5);
	// Getting Basic Information
	// ^^^^^^^^^^^^^^^^^^^^^^^^^
	//
	// We can print the name of the reference frame for this robot.
	ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());

	// We can also print the name of the end-effector link for this group.
	ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

	while(ros::ok())
	{
		int no_of_poses;
		std::cout << "Please, enter number of poses to run simulation (0 - 100): ";
		std::cin >> no_of_poses;
		if(std::cin.fail() || no_of_poses <= 0 || no_of_poses > 100)
		{
			std::cin.clear();
			std::cin.ignore(999,'\n');
			std::cout << "Please, enter valid input!! " << std::endl;
			continue;
		}
		while (no_of_poses != 0)
		{
			// Planning to a Pose goal
			// ^^^^^^^^^^^^^^^^^^^^^^^
			// We can plan a motion for this group to a random pose for the
			// end-effector.
			geometry_msgs::PoseStamped target_pose = group.getRandomPose();

			float x = target_pose.pose.position.x;
			float y = target_pose.pose.position.y;
			float z = target_pose.pose.position.z;

			// If goal pose is valid set it as target pose
			// else continue and get new random pose
			if(isPoseFeasible(x, y, z))
				group.setPoseTarget(target_pose);
			else
				continue;

			// Publish the goal pose for visulization
			display_goal_pose_publisher.publish(target_pose);

			// Now, we call the planner to compute the plan
			// and visualize it.
			// Note that we are just planning, not asking move_group
			// to actually move the robot.
			moveit::planning_interface::MoveGroup::Plan my_plan;
			bool plan_success = group.plan(my_plan);

			ROS_INFO("Visualizing plan (pose goal) %s",plan_success?"":"FAILED");
			/* Sleep to give Rviz time to visualize the plan. */
			sleep(5.0);

			// Visualizing plans
			// ^^^^^^^^^^^^^^^^^
			// Now that we have a plan we can visualize it in Rviz.  This is not
			// necessary because the group.plan() call we made above did this
			// automatically.  But explicitly publishing plans is useful in cases that we
			// want to visualize a previously created plan.
			if (plan_success)
			{
				ROS_INFO("Visualizing plan (again)");

				//moveit_msgs::DisplayTrajectory display_trajectory;
				moveit_msgs::DisplayTrajectory display_trajectory;
				display_trajectory.trajectory_start = my_plan.start_state_;
				display_trajectory.trajectory.push_back(my_plan.trajectory_);
				display_publisher.publish(display_trajectory);
				/* Sleep to give Rviz time to visualize the plan. */
				sleep(5.0);
			}

			// Moving to a pose goal
			// ^^^^^^^^^^^^^^^^^^^^^
			//
			// Given a plan, execute it while waiting for completion. Return true on success
			if(plan_success)
			{
				group.execute(my_plan);
				sleep(5.0);
			}
			no_of_poses -= 1;
		}
	}
	ros::shutdown();  
	return 0;
}
