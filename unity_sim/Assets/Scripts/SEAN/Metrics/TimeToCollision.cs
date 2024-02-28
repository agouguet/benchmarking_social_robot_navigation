using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TimeToCollision : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        
    }

    private void OnTriggerEnter(Collider hit)
    {
        if (hit.gameObject.tag.Equals("TTC"))
        {
            Vector3 closestPoint = hit.ClosestPoint(gameObject.transform.position);
            print(closestPoint);
        }

    }

    private void OnTriggerStay(Collider hit)
    {
        if (hit.gameObject.tag.Equals("TTC"))
        {
            Vector3 closestPoint = hit.ClosestPoint(gameObject.transform.position);
            print(closestPoint);
        }
    }
}
