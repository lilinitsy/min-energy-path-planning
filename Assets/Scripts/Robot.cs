using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Robot : MonoBehaviour
{
	Lidar lidar;
	// Use this for initialization
	void Start()
	{
		lidar = GetComponentInChildren<Lidar>();
	}
	
	// Update is called once per frame
	void Update()
	{
		
	}
}
