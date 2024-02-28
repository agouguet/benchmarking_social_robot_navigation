using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PositionAgent : MonoBehaviour
{
    public float desiredSpeed = Parameters.DESIRED_SPEED;
    public float maxSpeed = Parameters.MAX_VEL;
    public bool staticAgent = false;
    public float perception = Parameters.PERCEPTION_RADIUS_AGENT;
}
