using System.Collections;
using System.Collections.Generic;
using UnityEngine;


// CAN USE PHYSICS.OVERLAPBOX TO GET ALL COLLISIONS
public class QuadTree
{
	public QuadBox boundary = null;
	public QuadTree top_left = null;
	public QuadTree bottom_left = null;
	public QuadTree top_right = null;
	public QuadTree bottom_right = null;

	// the minimum width of the bounding box. Should make it a wee bit wider than the robots bounding box.
	public float minimum_half_dimension = 0.5f;
	// Is there an object in here?
	public bool clear = true;
	public bool has_children = false;

	public QuadTree()
	{
		top_left = null;
		bottom_left = null;
		top_right = null;
		bottom_right = null;
	}

	public QuadTree(float mhd)
	{
		top_left = null;
		bottom_left = null;
		top_right = null;
		bottom_right = null;
		minimum_half_dimension = mhd;
	}

	public QuadTree(QuadBox bound, float mhd = 0.5f)
	{
		top_left = null;
		bottom_left = null;
		top_right = null;
		bottom_right = null;
		boundary = bound;
		minimum_half_dimension = mhd;
	}


	public void build_quadtree()
	{
		if(!boundary.cell_clear())
		{
			Debug.Log("Not yet in subdivide if");
			clear = false;

			Debug.Log("Is top_left null?" + top_left == null);
			//top_left.boundary.print();
			// top_left null -> no children; check that it can still subdivide
			Debug.Log("Has children? " + has_children);
			Debug.Log("boundary.half_dimension: " + boundary.half_dimension);
			Debug.Log("2.0 * minimum_half_dimension: " + 2.0f * minimum_half_dimension);
			if(!has_children && boundary.half_dimension >=  2.0f * minimum_half_dimension)
			{
				Debug.Log("Right above subdivide");
				subdivide();
				top_left.build_quadtree();
				top_right.build_quadtree();
				bottom_left.build_quadtree();
				bottom_right.build_quadtree();
			}

			// top left null, but this node's boundary is too big to subdivide.
			else if(top_left == null && boundary.half_dimension < 2.0f * minimum_half_dimension)
			{
				return; 
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
											boundary.center.x - child_half_dimension,
											boundary.center.y,
											boundary.center.z + child_half_dimension);
		Vector3 top_right_center = new Vector3(
											boundary.center.x + child_half_dimension,
											boundary.center.y,
											boundary.center.z + child_half_dimension);
		Vector3 bottom_left_center = new Vector3(
											boundary.center.x - child_half_dimension,
											boundary.center.y,
											boundary.center.z - child_half_dimension);
		Vector3 bottom_right_center = new Vector3(
											boundary.center.x + child_half_dimension,
											boundary.center.y,
											boundary.center.z - child_half_dimension);			

		QuadBox top_left_box = new QuadBox(top_left_center, child_half_dimension);
		QuadBox top_right_box = new QuadBox(top_right_center, child_half_dimension);
		QuadBox bottom_left_box = new QuadBox(bottom_left_center, child_half_dimension);
		QuadBox bottom_right_box = new QuadBox(bottom_right_center, child_half_dimension);

		top_left = new QuadTree(top_left_box, minimum_half_dimension);
		top_right = new QuadTree(top_right_box, minimum_half_dimension);
		bottom_left = new QuadTree(bottom_left_box, minimum_half_dimension);
		bottom_right = new QuadTree(bottom_right_box, minimum_half_dimension);
		has_children = true;
	}

	public void draw()
	{
		Gizmos.color = Color.green;
		if(boundary != null)
		{
			float size = boundary.half_dimension * 2.0f;
			Gizmos.DrawWireCube(boundary.center, new Vector3(size, size, size));
		}

		if(top_left != null)
		{
			Debug.Log("top left not null");
			top_left.draw();
			top_right.draw();
			bottom_left.draw();
			bottom_right.draw();
		}

	}

}
