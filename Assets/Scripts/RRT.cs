﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;


enum RRTSTATUS
{
	TRAPPED,
	EXTENDED,
	REACHED,
	GOAL_REACHED
};


public class RRT
{
	public List<Node> nodes;
	public List<Node> leaf_nodes;
	public Vector3 global_goal;
	public int step_size;

	public RRT()
	{

	}

	public RRT(Vector3 g, int sz)
	{
		global_goal = g;
		step_size = sz;
	}


	public void build_rrt(Vector3 start, int iterations)
	{
		Node start_node = new Node(start);
		nodes.Add(start_node);
		leaf_nodes.Add(start_node);

		Vector3 begin_position = start;

		// iterations
		int z = 0;

		while(!goal_found())
		{
			z++;

			int random_choice = (int) (Random.value * 20.0f);
			Vector3 local_goal_position;
			Node begin_node;

			if(random_choice > 17 || Vector3.Distance(begin_position, global_goal) < step_size * iterations)
			{
				local_goal_position = global_goal;
				int closest_node = get_closest_node_to_point(global_goal);
				begin_node = nodes[closest_node];
				begin_position = begin_node.position;
			}

			else
			{
			//	local_goal_position = pick_local_goal_position(nodes[nodes.size() - 1].position, iterations);
			}


			//RRTSTATUS status = extend(local_goal_position, ref begin_node);
			

		}
	}


	RRTSTATUS extend(Vector3 goal_pos, ref Node begin_node)
	{
		Vector3 direction = goal_pos - begin_node.position;
		float distance = Vector3.Distance(goal_pos, begin_node.position);
		int layer_mask = 1 << 9;
		layer_mask = ~layer_mask;

		if(Vector3.Distance(begin_node.position, global_goal) < step_size)
		{
			Vector3 goal_direction = global_goal - begin_node.position;
			float goal_distance = Vector3.Distance(global_goal, begin_node.position);
			if(!Physics.Raycast(begin_node.position, goal_direction, goal_distance, layer_mask))
			{
				if(begin_node.children.Count > 0)
				{
					remove_from_leaf_list(begin_node);
				}

				Node global_goal_node = new Node(global_goal);
				begin_node.children.Add(global_goal_node);
				nodes.Add(global_goal_node);
				leaf_nodes.Add(global_goal_node);
				return RRTSTATUS.GOAL_REACHED;
			}
		}

		if(Physics.Raycast(begin_node.position, direction, distance, layer_mask))
		{
			return RRTSTATUS.TRAPPED;
		}

		if(begin_node.children.Count > 0)
		{
			remove_from_leaf_list(begin_node);
		}

		Node next_node = new Node(goal_pos);
		begin_node.children.Add(next_node);
		nodes.Add(next_node);
		leaf_nodes.Add(next_node);
		return RRTSTATUS.EXTENDED;
	}


	private bool goal_found()
	{
		for(int i = 0; i < nodes.Count; i++)
		{
			if(nodes[i].position == global_goal)
			{
				return true;
			}
		}

		return false;
	}


	private int get_closest_node_to_point(Vector3 point)
	{
		if(nodes.Count > 0)
		{
			int closest_index = 0;
			Node closest_node = nodes[0];
			float min_distance = Vector3.Distance(closest_node.position, point);

			for(int i = 0; i < nodes.Count; i++)
			{
				float dist = Vector3.Distance(nodes[i].position, point);
				if(dist < min_distance)
				{
					min_distance = dist;
					closest_node = nodes[i];
					closest_index = i;
				}
			}

			return closest_index;
		}
		
		return 0;
	}

	// Can ignore y, pick and x and z, and then follow up the norm of the hit object? hmm...
	/*private Vector3 pick_local_goal_position(Vector3 start)
	{
		int x = start.x + step_size
	}*/


	private void remove_from_leaf_list(Node node)
	{
		for(int i = 0; i < leaf_nodes.Count; i++)
		{
			if(leaf_nodes[i] == node)
			{
				leaf_nodes.Remove(node);
			}
		}
	}

}
