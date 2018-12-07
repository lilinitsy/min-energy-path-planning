using System.Collections;
using System.Collections.Generic;
using UnityEngine;

enum QUADRANT
{
	TOP_LEFT,
	BOTTOM_LEFT,
	TOP_RIGHT,
	BOTTOM_RIGHT
};


/*
	IMPORTANT:
	In cell_clear(), the check for collisions is done
		The half_dimension for y is treated as 100.0f there, but y values are completely
		disregarded everywhere else.
*/
public class QuadBox
{
	public Vector3 center;
	public float half_dimension;
	public bool clear = true;

	public QuadBox()
	{

	}

	// Overload for passing Vector2's
	public QuadBox(Vector2 c, float hdx)
	{
		center.x = c.x;
		center.y = 0.0f;
		center.z = c.y;
		half_dimension = hdx;
	}

	public QuadBox(Vector3 c, float hdx)
	{
		center = c;
		half_dimension = hdx;
	}

	// Overload for passing Vector2's
	public bool contains_point(Vector2 point)
	{
		bool left_edge = point.x >= center.x - half_dimension;
		bool right_edge = point.x <= center.x + half_dimension;
		bool bottom_edge = point.y >= center.z - half_dimension;
		bool top_edge = point.y <= center.z + half_dimension;

		return (left_edge && right_edge && bottom_edge && top_edge);
	}

	public bool contains_point(Vector3 point)
	{
		bool left_edge = point.x >= center.x - half_dimension;
		bool right_edge = point.x <= center.x + half_dimension;
		bool bottom_edge = point.y >= center.z - half_dimension;
		bool top_edge = point.y <= center.z + half_dimension;

		return (left_edge && right_edge && bottom_edge && top_edge);
	}


	public bool cell_clear()
	{
		int layer_mask = 1 << 9;
		layer_mask = ~layer_mask;

		Vector3 half_extents = new Vector3(half_dimension, 100.0f, half_dimension);

		Collider[] colliders = Physics.OverlapBox(center, half_extents, Quaternion.identity, layer_mask);
		
		for(int i = 0; i < colliders.Length; i++)
		{
			Debug.Log("Collider name: " + colliders[i].gameObject.name);
		}
		
		if(colliders.Length > 0)
		{
			Debug.Log("Colliders here\n");
			clear = false;
			return false;
		}
		
		clear = true;
		return true;
	}


	public void print()
	{
		Debug.Log("Center: " + center.ToString());
		Debug.Log("half dimension: " + half_dimension);
	}
}
