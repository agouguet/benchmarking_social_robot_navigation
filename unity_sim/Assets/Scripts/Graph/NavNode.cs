using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.VisualScripting;


#if UNITY_EDITOR
using UnityEditor;
#endif

[ExecuteInEditMode]
public class NavNode : MonoBehaviour
{

    public bool createNode = false;

    public bool createEdge = false;

    public bool is_goal = false;

    public List<GameObject> neighbor = new List<GameObject>();

    public MeshRenderer render;

    public float radius;

    public GameObject nodesGO;

    public GameObject prefab;

    public const float NODE_RADIUS = 0.3f;

    private GameObject CreateNode(GameObject example)
    {
        var GO = GameObject.Instantiate(prefab);
        GO.transform.position = example.transform.position;
        GO.transform.parent = nodesGO.transform; 
        GO.name = "Node " + (GameObject.FindObjectsOfType<NavNode>().Length);
        GO.GetComponent<NavNode>().radius = NODE_RADIUS;
        GO.GetComponent<NavNode>().neighbor = new List<GameObject>();

        this.neighbor.Add(GO);
        GO.GetComponent<NavNode>().neighbor.Add(gameObject);

        return GO.gameObject;
    }

    // Update is called once per frame
    void Update()
    {
        #if UNITY_EDITOR

            
            if (Application.isEditor)
            {
                #region Editor

                if (render == null)
                    render = GetComponent<MeshRenderer>();

                render.enabled = true;
                transform.localScale = Vector3.one * radius * 2;

                if (createNode)
                {
                    createNode = false;

                    var GO = this.CreateNode(gameObject);
                    Selection.activeGameObject = GO;
                }

                if (createEdge)
                {
                    createEdge = false;

                    GameObject[] nodes = Selection.gameObjects;

                    GameObject node1 = nodes[0];
                    GameObject node2 = nodes[1];

                    if(node1.GetComponent<NavNode>() != null && node2.GetComponent<NavNode>() != null)
                    {
                        if(!node1.GetComponent<NavNode>().neighbor.Contains(node2))
                            node1.GetComponent<NavNode>().neighbor.Add(node2);
                        if(!node2.GetComponent<NavNode>().neighbor.Contains(node1))
                            node2.GetComponent<NavNode>().neighbor.Add(node1);
                    }
                }

                #endregion
            }
        #endif
        
    }

    void OnDrawGizmosSelected()
    {
        foreach(GameObject neigh in neighbor){
            Gizmos.color = Color.blue;
            Gizmos.DrawLine(transform.position, neigh.transform.position);
        }
    }
}
