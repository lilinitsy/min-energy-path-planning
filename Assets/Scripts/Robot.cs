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
	private RRT rrt;
	private Vector3 local_goal;

	private List<Node> tmp_gizmo_drawer;

	private int hacky_counter = 0;

	// Use this for initialization
	void Start()
	{
		lidar = GetComponentInChildren<Lidar>();
		tmp_gizmo_drawer = new List<Node>();
		k = 1.0f + k;
		gravity = Physics.gravity.y;
		local_goal = transform.position;
		rrt = new RRT(goal, 1.0f);
		rrt.forward = transform.forward;
		rrt.build_rrt(transform.position, 1000);
		
		global_path = AStar(goal);
	}
	

	// Update is called once per frame
	void Update()
	{
		Node[ , ] sample_possible_nodes = sample_lidar();
		Vector3[ , ] sample_hit_normals = sample_lidar_normals();
		List<Node> candidate_nodes = new List<Node>();
		hacky_counter++;
		if(Vector2.Distance(new Vector2(local_goal.x, local_goal.z), new Vector2(transform.position.x, transform.position.z)) < 0.1f && hacky_counter > 1)
		{
			tmp_gizmo_drawer.Clear();

			for(int i = 1; i < sample_possible_nodes.GetLength(0) - 1; i++)
			{
				for(int j = 1; j < sample_possible_nodes.GetLength(1) - 1; j++)
				{
					// check for position
					if(sample_possible_nodes[i, j].position.x != Mathf.Infinity && sample_possible_nodes[i, j].position.y != Mathf.Infinity)
					{
						Node possible_node = new Node(sample_possible_nodes[i, j].position);
						Vector3 possible_node_normal = sample_hit_normals[i, j];
						float angle = estimate_steepness_angle(possible_node, sample_possible_nodes, i, j);
				
						if(Mathf.Abs(angle) < 90.0f && Mathf.Abs(possible_node_normal.x) < 0.25f && Mathf.Abs(possible_node_normal.z) < 0.25f)
						{
							possible_node.calculate_energy(transform.position, k, mass, angle);
							candidate_nodes.Add(possible_node);
						}
					}
				}
			}

			List<Node> minimum_dPI_dYQ_nodes = candidate_nodes;
			List<float> dist_sorted_vecs = new List<float>();

			for(int i = 0; i < candidate_nodes.Count; i++)
			{
				dist_sorted_vecs.Add(Vector3.Distance(goal, candidate_nodes[i].position));
			}

			minimum_dPI_dYQ_nodes.Sort((n1, n2) => Mathf.Abs(n1.dPI_dY).CompareTo(Mathf.Abs(n2.dPI_dY)));
			dist_sorted_vecs.Sort();
			for(int i = 0; i < candidate_nodes.Count; i++)
			{
				//Debug.Log("Node least squared partials: " + candidate_nodes[i].least_square_value + "\tPosition: " + candidate_nodes[i].position.ToString("F4"));
			}

			Node minimum_node = get_minimum_PE_node(minimum_dPI_dYQ_nodes);
			local_goal = minimum_node.position;
			Debug.Log("*****");
			Debug.Log("MINIMUM NODE: " +  minimum_node.position.ToString("F4"));
			Debug.Log("*****");

			rotate_routine(minimum_node);
		}

		else
		{
			movement_routine();
		}
	}


	private void movement_routine()
	{
		transform.position = Vector3.MoveTowards(transform.position, local_goal, 0.05f);
		//Debug.Log("Distance: " + Vector3.Distance())
	}


	private void rotate_routine(Node minimum_node)
	{
		Quaternion target_rotation = Quaternion.LookRotation(minimum_node.position - transform.position);
		transform.rotation = Quaternion.RotateTowards(transform.rotation, target_rotation, 1.0f);
	}


	private struct NodeWithScore
	{
		public Node node;
		public float score;

		public NodeWithScore(Node n, float s)
		{
			node = n;
			score = s;
		}
	}

	private Node get_minimum_PE_node(List<Node> minimum_dPI_dYQ_nodes)
	{
		List<NodeWithScore> scores = new List<NodeWithScore>();

		for(int i = 0; i < minimum_dPI_dYQ_nodes.Count; i++)
		{
			float score = minimum_dPI_dYQ_nodes[i].dPI_dY * Vector3.Distance(goal, minimum_dPI_dYQ_nodes[i].position);
			scores.Add(new NodeWithScore(minimum_dPI_dYQ_nodes[i], score));
		}

		scores.Sort((n1, n2) => Mathf.Abs(n1.score).CompareTo(Mathf.Abs(n2.score)));

		Debug.Log("Minimum node dPI/dY: " + scores[0].node.dPI_dY + "\tDistance: " + Vector3.Distance(goal, scores[0].node.position));
		Debug.Log("Minimum node dPI/dY: " + scores[1].node.dPI_dY + "\tDistance: " + Vector3.Distance(goal, scores[1].node.position));
		Debug.Log("Minimum node dPI/dY: " + scores[2].node.dPI_dY + "\tDistance: " + Vector3.Distance(goal, scores[2].node.position));
		Debug.Log("Minimum node dPI/dY: " + scores[3].node.dPI_dY + "\tDistance: " + Vector3.Distance(goal, scores[3].node.position));

		Debug.Log("Minimum node least square: " + scores[0].node.least_square_value + "\tDistance: " + Vector3.Distance(goal, scores[0].node.position));
		Debug.Log("Minimum node least square2: " + scores[1].node.least_square_value + "\tDistance: " + Vector3.Distance(goal, scores[1].node.position));
		Debug.Log("Minimum node least square: " + scores[2].node.least_square_value + "\tDistance: " + Vector3.Distance(goal, scores[2].node.position));
		Debug.Log("Minimum node least square: " + scores[3].node.least_square_value + "\tDistance: " + Vector3.Distance(goal, scores[3].node.position));

		tmp_gizmo_drawer.Add(scores[0].node);
		tmp_gizmo_drawer.Add(scores[1].node);
		tmp_gizmo_drawer.Add(scores[2].node);
		tmp_gizmo_drawer.Add(scores[3].node);

		return scores[0].node;
	}




	private float estimate_steepness_angle(Node possible_node, Node[ , ] current_possible_nodes, int i, int j)
	{
		float direction_angle = Vector3.SignedAngle(transform.position, possible_node.position, transform.forward);

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
		}
		
		return angle;
	}


	void OnDrawGizmosSelected()
	{
		Gizmos.color = Color.cyan;
		for(int i = 0; i < rrt.nodes.Count; i++)
		{
			Gizmos.DrawSphere(rrt.nodes[i].position, 0.1f);
		}

		Gizmos.color = Color.red;

		for(int i = 0; i < tmp_gizmo_drawer.Count; i++)
		{
			Gizmos.DrawCube(tmp_gizmo_drawer[i].position, new Vector3(0.1f, 0.1f, 0.1f));
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

	private Vector3[ , ] sample_lidar_normals()
	{
		Vector3[ , ] normals = new Vector3[lidar.hit_points.GetLength(0) / 3, lidar.hit_points.GetLength(1)];

		for(int i = 0; i < normals.GetLength(0); i++)
		{
			for(int j = 0; j < normals.GetLength(1); j++)
			{
				Vector3 norm = lidar.hit_normals[i * 3, j];
				normals[i, j] = norm;
			}
		}

		return normals;
	}

	private struct AStarNode
	{
		public Node node;
		public float gscore;
		public float fscore;

		public AStarNode(Node n, float g, float f)
		{
			node = n;
			gscore = g;
			fscore = f;
		}
	}

	private struct NodeMap
	{
		public Node parent;
		public Node child;

		public NodeMap(Node p, Node c)
		{
			parent = p;
			child = c;
		}
	}


	// todo: check continue's
	private List<Node> AStar(Vector3 goal)
	{
		List<Node> closed_set = new List<Node>();
		List<AStarNode> open_set = new List<AStarNode>();
		List<NodeMap> came_from = new List<NodeMap>();
		//List<AStarNode> node_list = new List<AStarNode>();

		open_set.Add(new AStarNode(rrt.nodes[0], 0, heuristic_cost(rrt.nodes[0].position, goal)));

		// add everything into open_set, with infinite g and f score
		for(int i = 1; i < rrt.nodes.Count; i++)
		{
			open_set.Add(new AStarNode(rrt.nodes[0], Mathf.Infinity, Mathf.Infinity));
		}

		AStarNode current_node = get_lowest_fscore_node(open_set);

		while(open_set.Count > 0)
		{
			current_node = get_lowest_fscore_node(open_set);
			if(current_node.node.position == goal)
			{
				return reconstruct_path(came_from, current_node);
			}

			open_set.Remove(current_node);
			closed_set.Add(current_node.node);

			for(int i = 0; i < current_node.node.children.Count; i++)
			{
				Node child_node = current_node.node.children[i];

				for(int j = 0; j < closed_set.Count; j++)
				{
					if(closed_set[j].position == child_node.position)
					{
						continue;
					}
				}

				float tentative_gscore = current_node.gscore + heuristic_cost(current_node.node.position, child_node.position);
				bool child_in_open_set = false;
				int child_index = 0;

				for(int j = 0; j < open_set.Count; j++)
				{
					if(child_node == open_set[i].node || child_node.position == open_set[i].node.position)
					{
						child_in_open_set = true;
						child_index = i;
						break;
					}
				}

				if(!child_in_open_set)
				{
					open_set.Add(new AStarNode(child_node, tentative_gscore, tentative_gscore + heuristic_cost(child_node.position, goal)));
				}

				else if(!(tentative_gscore > open_set[child_index].gscore))
				{
					came_from.Add(new NodeMap(current_node.node, child_node));
				}
			}
		}

		return reconstruct_path(came_from, current_node);
	}


	private List<Node> reconstruct_path(List<NodeMap> came_from, AStarNode current_node)
	{
		List<Node> path = new List<Node>();

		for(int i = 0; i < came_from.Count; i++)
		{
			path.Add(came_from[i].parent);
		}


		return new List<Node>();
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


	private AStarNode get_lowest_fscore_node(List<AStarNode> nodes)
	{
		float min = Mathf.Infinity;
		int min_index = 0;

		for(int i = 0; i < nodes.Count; i++)
		{
			if(nodes[i].fscore <= min)
			{
				min = nodes[i].fscore;
				min_index = i;
			}
		}

		return nodes[min_index];
	}
}
