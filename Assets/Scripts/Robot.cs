using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Robot : MonoBehaviour
{
	public Vector3 goal;
	private QuadTreeRun quadtree;
	private Lidar lidar;

	private List<Node> global_path;

	// Use this for initialization
	void Start()
	{
		lidar = GetComponentInChildren<Lidar>();
		quadtree = GetComponentInChildren<QuadTreeRun>();
		global_path = AStar(quadtree.quadtree, goal);
	}
	
	// Update is called once per frame
	void Update()
	{
		
	}

	private List<Node> AStar(QuadTree qt, Vector3 goal)
	{
		List<Node> path = new List<Node>();
		// closed set
		List<Node> evaluated_nodes = new List<Node>();
		// open set
		List<Node> open_nodes = new List<Node>();
		List<QuadTree> leaf_quadtrees = qt.list_leaf_nodes();
		// cost of getting from start node to node n
		List<float> gscore = new List<float>();
		List<float> fscore = new List<float>();
		Node start = new Node(transform.position);

		open_nodes.Add(start);
		gscore.Add(0);

		return path;
	}
}
