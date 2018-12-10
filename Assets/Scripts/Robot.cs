using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

public class Robot : MonoBehaviour
{
	public float k;
	public float mass;

	private float total_energy = 0.0f;
	private float gravity;
	public Vector3 goal;
	public QuadTreeRun quadtree;
	private Lidar lidar;

	private List<Node> global_path;

	// Use this for initialization
	void Start()
	{
		lidar = GetComponentInChildren<Lidar>();
		//global_path = AStar(quadtree.quadtree, goal);
		k = 1.0f + k;
		gravity = Physics.gravity.y;
	}
	

	/*
		7PM 8/12:
		Don't work on global path finding, just do local
	
	 */

	// Update is called once per frame
	void Update()
	{
		//global_path = AStar(quadtree.quadtree, new Vector3(5, 5, 5));
		// ASSIGN NODES AND THEIR WEIGHTS ("energy")
		// THEN TRY TO FIND THE dPI/dQ & dPI/dY MIN ENERGY MAX DISTANCE INTERSECTION
		Node[ , ] sample_possible_nodes = sample_lidar();
		List<Node> candidate_nodes = new List<Node>();

		for(int i = 1; i < sample_possible_nodes.GetLength(0) - 1; i++)
		{
			for(int j = 1; j < sample_possible_nodes.GetLength(1) - 1; j++)
			{
				// check for position
				if(sample_possible_nodes[i, j].position.x != Mathf.Infinity)
				{
					Node possible_node = new Node(sample_possible_nodes[i, j].position);
					float angle = estimate_steepness_angle(possible_node, sample_possible_nodes, i, j);
					possible_node.calculate_energy(transform.position, k, mass, angle);
					candidate_nodes.Add(possible_node);
				
				}
			}
		}

	}


	private float estimate_steepness_angle(Node possible_node, Node[ , ] current_possible_nodes, int i, int j)
	{
		//Vector3 direction_robot_to_point = Vector3.Normalize(transform.position - possible_node.position);
		//Debug.Log("direction vector: " + direction_robot_to_point.ToString("F5"));

		float angle = Vector3.SignedAngle(transform.position, possible_node.position, transform.forward);
		Debug.Log("Transform.forward: " + transform.forward.ToString("F4"));
		Debug.Log("Angle between position and the node position " + possible_node.position.ToString("F4") + ": " + angle);

		// Up 1, left 1
		/*if(Vector3.SignedAngle(transform.position, direction_robot_to_point, transform.forward) < 45)
		{
			float angle = Vector3.Angle(possible_node.position, current_possible_nodes[i - 1, j + 1].position);
		}*/
		
		

		return 1.0f;
	}

	
	private Node[ , ] sample_lidar()
	{
		Node[ , ] sample_points = new Node[lidar.hit_points.GetLength(0) / 3, lidar.hit_points.GetLength(1)];
		
		for(int i = 0; i < sample_points.GetLength(0); i++)
		{
			for(int j = 0; j < sample_points.GetLength(1); j++)
			{
				Node node = new Node(lidar.hit_points[i * 3, j]);
				sample_points[i, j] = node;		
			}
		}

		return sample_points;
	}

	// MIGHT JUST WANT TO DO AN RRT
	private List<Node> AStar(QuadTree qt, Vector3 goal)
	{
		List<Node> path = new List<Node>();
		// closed set
		List<Node> evaluated_nodes = new List<Node>();
		// open set
		List<Node> open_nodes = new List<Node>();
		List<QuadTree> leaf_quadtrees = qt.list_leaf_nodes();
		// cost of getting from start node to node n
		List<float> gscores = new List<float>();
		List<float> fscores = new List<float>();
		Node start = new Node(transform.position);

		open_nodes.Add(start);
		gscores.Add(0);
		fscores.Add(heuristic_cost(start.position, goal));

		for(int i = 0; i < leaf_quadtrees.Count; i++)
		{
			Node quadtree_node = new Node(leaf_quadtrees[i].boundary.center);
			open_nodes.Add(quadtree_node);
			gscores.Add(Mathf.Infinity);
			fscores.Add(heuristic_cost(quadtree_node.position, goal));
		}

		while(open_nodes.Count > 0)
		{
			Node current_node = open_nodes[get_lowest_fscore_node(fscores)];

			if(current_node.position == goal)
			{

			}

			open_nodes.Remove(current_node);
			evaluated_nodes.Add(current_node);


		}

		return path;
	}


	private float heuristic_cost(Vector3 start, Vector3 goal)
	{
		float q = Mathf.Sqrt(Mathf.Abs((start.x - goal.x) + (start.z - goal.z)));
		float y = Mathf.Abs(start.y - goal.y);

		float energy_heuristic = 0.5f * k * q * q
								+ 0.5f * k * y * y
								- mass * gravity * k * y * Mathf.Cos(Vector3.Angle(start, goal))
								+ 0.5f * k * (q * q + y * y);
		return energy_heuristic;
	}


	private int get_lowest_fscore_node(List<float> fscores)
	{
		float minval = fscores.Min();
		return fscores.IndexOf(minval);
	}
}
