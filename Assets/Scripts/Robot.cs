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
	private Lidar lidar;

	private List<Node> global_path;


	private List<Node> tmp_gizmo_drawer;

	// Use this for initialization
	void Start()
	{
		lidar = GetComponentInChildren<Lidar>();
		tmp_gizmo_drawer = new List<Node>();
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
		tmp_gizmo_drawer.Clear();

		for(int i = 1; i < sample_possible_nodes.GetLength(0) - 1; i++)
		{
			for(int j = 1; j < sample_possible_nodes.GetLength(1) - 1; j++)
			{
				// check for position
				if(sample_possible_nodes[i, j].position.x != Mathf.Infinity && sample_possible_nodes[i, j].position.y != Mathf.Infinity)
				{
					Node possible_node = new Node(sample_possible_nodes[i, j].position);
					float angle = estimate_steepness_angle(possible_node, sample_possible_nodes, i, j);
					possible_node.calculate_energy(transform.position, k, mass, angle);
					candidate_nodes.Add(possible_node);
				}
			}
		}

		for(int i = 0; i < candidate_nodes.Count; i++)
		{
			
		}
	}


	private float estimate_steepness_angle(Node possible_node, Node[ , ] current_possible_nodes, int i, int j)
	{
		float direction_angle = Vector3.SignedAngle(transform.position, possible_node.position, transform.forward);
		Debug.Log("Direction angle: " + direction_angle);

		// angle = change in y / 
		Vector2 previous_node_q;
		Vector2 node_q = new Vector2(possible_node.position.x, possible_node.position.z);

		if(direction_angle >= 0.0f && direction_angle < 45.0f)
		{
			previous_node_q = new Vector2(current_possible_nodes[i, j + 1].position.x, current_possible_nodes[i, j + 1].position.z);
		}

		else if(direction_angle >= 45.0f)
		{
			previous_node_q = new Vector2(current_possible_nodes[i + 1, j + 1].position.x, current_possible_nodes[i + 1, j + 1].position.z);
		}

		else if(direction_angle < 0.0f && direction_angle > -45.0f)
		{
			previous_node_q = new Vector2(current_possible_nodes[i, j - 1].position.x, current_possible_nodes[i, j - 1].position.z);
		}

		else
		{
			previous_node_q = new Vector2(current_possible_nodes[i - 1, j - 1].position.x, current_possible_nodes[i - 1, j - 1].position.z);
		}


		float qdist = Vector2.Distance(previous_node_q, node_q);
		float ydist = transform.position.y - 0.5f - possible_node.position.y;
		float angle = 0.0f;
		
		if(qdist > 0.0f)
		{
			angle = Mathf.Tan(ydist / qdist) * Mathf.Rad2Deg;
			if(angle > 2.0f)
			{
				Debug.Log("Possible node y: " + possible_node.position.y);
				tmp_gizmo_drawer.Add(possible_node);
				Debug.Log("Angle: " + angle);
			}
		}
		
		return angle;
	}


	void OnDrawGizmosSelected()
	{
		Gizmos.color = Color.cyan;
		for(int i = 0; i < tmp_gizmo_drawer.Count; i++)
		{
			Gizmos.DrawSphere(tmp_gizmo_drawer[i].position, 0.1f);
		}
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
	private List<Node> AStar(Vector3 goal)
	{
		List<Node> path = new List<Node>();
		// closed set
		List<Node> evaluated_nodes = new List<Node>();
		// open set
		List<Node> open_nodes = new List<Node>();
		// cost of getting from start node to node n
		List<float> gscores = new List<float>();
		List<float> fscores = new List<float>();
		Node start = new Node(transform.position);

		open_nodes.Add(start);
		gscores.Add(0);
		fscores.Add(heuristic_cost(start.position, goal));

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
