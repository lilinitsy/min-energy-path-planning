using System.Collections;
using System.Collections.Generic;
using UnityEngine;


enum RRT_STATUS
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


	public void build_rrt(Vector3 start, QuadTree quadtree, int iterations)
	{
		Node start_node = new Node(start);
		nodes.Add(start_node);
		leaf_nodes.Add(start_node);

		Vector3 begin_position = start;

		// iterations
		int z = 0;

		while(z < 1000)
		{
			z++;

			int random_choice = (int) (Random.value * 20.0f);
			Vector3 local_goal_position;
			Node begin_node;

			if(random_choice < 17 || Vector3.Distance(begin_position, global_goal) < step_size * iterations)
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

		}
	}


	private int get_closest_node_to_point(Vector3 point)
	{
		return 1;
	}


	private Vector3 pick_local_goal_position(Vector3 start, int iterations, QuadTree quadtree)
	{
		return new Vector3(0, 0, 0);
	}

}
