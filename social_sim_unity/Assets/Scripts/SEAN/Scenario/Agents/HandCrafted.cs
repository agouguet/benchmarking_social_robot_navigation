// Copyright (c) 2021, Members of Yale Interactive Machines Group, Yale University,
// Nathan Tsoi
// All rights reserved.
// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree. 

using System.Collections.Generic;
using UnityEngine;

namespace SEAN.Scenario.Agents
{
    public class HandCrafted : BaseAgentManager
    {
        public float SPAWN_HEIGHT = 0;
        public float WAYPOINT_DIST = 1.5f;

        public List<IVI.INavigable> agents;
        public List<Trajectory.TrackedGroup> groups;

        public GameObject agentPrefab;

        private GameObject agentsGO;
        private List<List<Pose>> positions;
        private Dictionary<IVI.INavigable, List<Pose>> agentGoals;

        public Pose openGroupLocation = Pose.identity;

        public string scenario_name
        {
            get
            {
                return "HandCrafted";
            }
        }

        protected override void Awake()
        {
            base.Awake();
            agentGoals = new Dictionary<IVI.INavigable, List<Pose>>();
        }

        void Update()
        {
            // UnityEngine.Debug.Log(current);
            // no need to plan if we only have static groups
            foreach (var agent in agents)
            {
                if (agent.CloseEnough())
                {
                    // Set the next goal
                    agent.InitDest(agentGoals[agent][2].position);
                    Pose currentGoal = agentGoals[agent][0];
                    agentGoals[agent].RemoveAt(0);
                    agentGoals[agent].Add(currentGoal);
                }
            }
            
        }

        public void NewScenario(GameObject socialSituationEnv, GameObject spawnLocations)
        {
            // Must clear before registering the new group
            Clear();

            // get agents game object
            foreach (Transform transform in socialSituationEnv.transform)
            {
                if (transform.gameObject.name == "Agents")
                {
                    agentsGO = transform.gameObject;
                }
            }

            int i = 0;
            foreach (Transform agentPositions in spawnLocations.transform)
            {
                List<Pose> agentPosition = new List<Pose>();
                foreach (Transform pos in agentPositions.transform)
                {
                    Vector3 temp = pos.position;
                    temp.y = 0;
                    agentPosition.Add(new Pose(temp, pos.rotation));
                }

                float desiredSpeed = agentPositions.GetComponent<PositionAgent>().desiredSpeed;
                float maxSpeed = agentPositions.GetComponent<PositionAgent>().maxSpeed;

                IVI.INavigable agent = SpawnAgent("Agent_" + (i+1), desiredSpeed, maxSpeed, agentPosition[0]);
                agentGoals.Add(agent, agentPosition);
                agent.InitDest(agentPosition[1].position);
                i++;
            }
        }

        void Clear()
        {
            if (!agentsGO) { return; }
            agents = new List<IVI.INavigable>();
            groups = new List<Trajectory.TrackedGroup>();
            foreach (Transform child in agentsGO.transform)
            {
                GameObject.Destroy(child.gameObject);
            }
            openGroupLocation = Pose.identity;
            agentGoals.Clear();
        }

        IVI.INavigable SpawnAgent(string name, float desiredSpeed, float maxSpeed, Pose pose)
        {
            var sfRandom = Instantiate(agentPrefab, Vector3.zero, Quaternion.identity);
            IVI.INavigable agent = sfRandom.GetComponentInChildren<IVI.INavigable>();
            agent.name = name;
            agent.desiredSpeed = desiredSpeed;
            agent.maxSpeed = maxSpeed;
            agent.transform.position = pose.position;
            agent.transform.rotation = pose.rotation;
            agent.transform.parent = agentsGO.transform;
            agents.Add(agent);
            return agent;
        }

        void SpawnGroup(Pose groupCenter)
        {
            IVI.GroupDataLoader.GroupData group = IVI.GroupDataLoader.groupData[Random.Range(0, IVI.GroupDataLoader.groupData.Count)];
            int openGroupIdx = Random.Range(0, group.pos.Count);
            for (int i = 0; i < group.pos.Count; i++)
            {

                float angle = Mathf.Atan(group.pos[i].x / group.pos[i].z);
                if (group.pos[i].z > 0)
                {
                    angle += Mathf.PI;
                }
                Quaternion rotation = Quaternion.Euler(0, angle * Mathf.Rad2Deg, 0);
                Pose pose = new Pose(groupCenter.position + group.pos[i], rotation);
                if (i == openGroupIdx)
                {
                    openGroupLocation = pose;
                }
                // else
                // {
                //     SpawnAgent("Agent_" + i, pose);
                // }
            }
        }

        void SpawnAgents()
        {
            // for (int i = 0; i < positions.Count; i++)
            // {
            //     Pose spawnPose = positions[i][0];
            //     IVI.INavigable agent = SpawnAgent("Agent_" + (i+1), spawnPose);
            //     agentGoals.Add(agent, positions[i]);
            //     agent.InitDest(positions[i][1].position);
            // }
            
            
            // if (situation == PedestrianBehavior.SocialSituation.JoinGroup || situation == PedestrianBehavior.SocialSituation.LeaveGroup)
            // {
            //     foreach (Pose pose in positions)
            //     {
            //         SpawnGroup(pose);
            //     }
            //     return;
            // }
        }
    }
}