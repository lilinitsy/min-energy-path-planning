using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Node
{
	public Vector3 position;
	public float energy = 0.0f;
	public float dPI_dQ = 0.0f;
	public float dPI_dY = 0.0f;
	public List<Node> children;

	public Node()
	{

	}

	public Node(Vector3 p)
	{
		position = p;
	}

	
	public void calculate_energy(Vector3 point, float k, float mass, float angle)
	{
		float x = point.x - position.x;
		float y = point.y - position.y;
		float z = point.z - position.z;
		float q = Mathf.Sqrt(x * x + z * z);

		float u_potential = 0.5f * k * (q * q) 
							+ 0.5f * k * (y * y);
		float v_potential = -1.0f * mass * Physics.gravity.y * y * Mathf.Cos(angle)
							- mass * Physics.gravity.y * q;
	
		dPI_dQ = k * q - mass * Physics.gravity.y;
		dPI_dY = k * y - mass * Physics.gravity.y * Mathf.Cos(angle);

		energy = u_potential + v_potential;
	}
}
