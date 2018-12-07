using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Lidar : MonoBehaviour
{
	public float degree_sweep_increment = 3.0f;
	// Start angle for initial pass
	public float max_height_angle = 15.0f;
	// maximum distance for which a hit can be registered
	public float max_range = 10.0f;
	// number of 360 degree passes, each at a varying angle
	public int number_vertical_passes = 10;

	// i + 1 is vertical, j + 1 is horizontal
	public Vector3[ , ] hit_points;

	// Use this for initialization
	void Start()
	{
		int horizontal_points = (int) (360.0f / degree_sweep_increment);
		hit_points = new Vector3[number_vertical_passes, horizontal_points];
	}
	
	// Update is called once per frame
	void Update()
	{
		Quaternion side_rotation = Quaternion.AngleAxis(degree_sweep_increment, transform.forward);
		Quaternion up_rotation = Quaternion.AngleAxis(-degree_sweep_increment, transform.right);

		Ray ray = new Ray();
		ray.origin = transform.position;
		ray.direction = transform.forward;
		RaycastHit hit;

		int horizontal_passes = (int) (360.0f / degree_sweep_increment);
		for(int i = 0; i < number_vertical_passes; i++)
		{
			for(int j = 0; j < horizontal_passes; j++)
			{
				if(Physics.Raycast(ray, out hit, max_range))
				{
					hit_points[i, j] = hit.point;
				}

				else
				{
					hit_points[i, j] = new Vector3(Mathf.Infinity, Mathf.Infinity, Mathf.Infinity);
				}
				
				ray.direction = side_rotation * ray.direction;
				Debug.Log("ray direction: " + ray.direction.ToString("F4"));
				Debug.Log("i, j: " + i + " " + j);
			}
			
			ray.direction = up_rotation * ray.direction;
		}
	}
	
	// Drawing the gizmos will be very slow
	void OnDrawGizmosSelected() 
	{
		Gizmos.color = Color.blue;
		int horizontal_passes = (int) (360.0f / degree_sweep_increment);
		Debug.Log("Lidar gizmos");
		for(int i = 0; i < number_vertical_passes; i++)
		{
			for(int j = 0; j < horizontal_passes; j++)
			{
				Gizmos.DrawSphere(hit_points[i, j], 0.1f);
			}
		}
	}
}
