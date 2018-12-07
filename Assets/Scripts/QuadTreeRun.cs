using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class QuadTreeRun : MonoBehaviour
{
	public QuadTree quadtree;
	private float half_dimension_y = 100.0f;
	// Use this for initialization
	void Start()
	{
		QuadBox bound = new QuadBox(new Vector3(0, 1, 0), 10.0f);
		float mhd = 0.5f;

		quadtree = new QuadTree(bound, mhd);
		quadtree.print();
		quadtree.build_quadtree();
	}

	// Update is called once per frame
	void Update()
	{
		
	}

	void OnDrawGizmos()
	{
		quadtree.draw();
	}
}
