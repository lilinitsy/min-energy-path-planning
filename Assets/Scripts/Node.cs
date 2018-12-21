using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Node
{
	public Vector3 position;
	public float energy = 0.0f;
	public float dPI_dQ = 0.0f;
	public float dPI_dY = 0.0f;
	public float least_square_value = 0.0f;
	public List<Node> children;

	public Node()
	{
		position = new Vector3(0.0f, 0.0f, 0.0f);
		children = new List<Node>();
	}

	public Node(Vector3 p)
	{
		children = new List<Node>();
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
		float v_potential = -1.0f * mass * Physics.gravity.y * y * Mathf.Sin(angle)
							- mass * Physics.gravity.y * q * Mathf.Cos(angle);
	
		dPI_dQ = k * q - mass * Physics.gravity.y * Mathf.Cos(angle);
		dPI_dY = k * y - mass * Physics.gravity.y * Mathf.Sin(angle);
		
		least_square_value = dPI_dQ + dPI_dY * dPI_dY;
		energy = u_potential + v_potential;
	}
}
