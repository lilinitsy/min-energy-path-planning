using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Node
{
	public Vector3 position;
	public List<Node> children;

	public Node()
	{

	}

	public Node(Vector3 p)
	{
		position = p;
	}
}
