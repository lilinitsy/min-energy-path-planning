using System.Collections;
using System.Collections.Generic;
using UnityEngine;


// CAN USE PHYSICS.OVERLAPBOX TO GET ALL COLLISIONS
public class QuadTree
{
	public QuadBox boundary = null;
	public QuadBox top_left = null;
	public QuadBox bottom_left = null;
	public QuadBox top_right = null;
	public QuadBox bottom_right = null;

	// the minimum width of the bounding box. Should make it a wee bit wider than the robots bounding box.
	public float minimum_half_dimension = 0.5f;
	// Is there an object in here?
	public bool clear = true;

	public QuadTree()
	{

	}

	public QuadTree(float mhd)
	{
		minimum_half_dimension = mhd;
	}

	public QuadTree(QuadBox bound, float mhd = 0.5f)
	{
		boundary = bound;
		minimum_half_dimension = mhd;
	}


	public void build_quadtree()
	{
		if(!boundary.cell_clear())
		{
			// top_left null -> no children; check that it can still subdivide
			if(top_left == null && boundary.half_dimension >=  2.0f * minimum_half_dimension)
			{
				
			}
		}
	}

	/*
	QUADRANT get_quadrant_containing_point(Vector3 point)
	{

	}*/


	public void print()
	{
		Debug.Log("Boundary: ");
		boundary.print();
	}


	private void subdivide()
	{
		float child_half_dimension = boundary.half_dimension / 2.0f;
		Vector3 top_left_center = new Vector3(
											boundary.center.x - boundary.half_dimension,
											boundary.center.y,

		)
	}



}
