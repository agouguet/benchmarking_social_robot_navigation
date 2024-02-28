// Copyright (c) 2021, Members of Yale Interactive Machines Group, Yale University,
// Nathan Tsoi
// All rights reserved.
// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree. 

using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace SEAN.Scenario.PedestrianBehavior
{
    public class HandCrafted : Base
    {
        Agents.HandCrafted agentManager;
        // Top level situation in the environment
        GameObject socialSituationEnv;
        // Top level of handcrafted situations, named with the same name as the enum
        GameObject spawnLocations;

        public override string scenario_name
        {
            get
            {
                return "HandCrafted";
            }
        }

        public void Start()
        {
            base.Start();
            NewScenario();
        }

        public void NewScenario()
        {
            foreach (Transform transform in pedestrianControl.transform)
            {
                if (transform.name == "HandcraftedSocialSituations")
                {
                    socialSituationEnv = transform.gameObject;
                    break;
                }
            }
            if (socialSituationEnv == null)
            {
                throw new System.Exception("Could not find social situations game object in environment");
            }
            socialSituationEnv.SetActive(true);

            foreach (Transform t1 in pedestrianControl.transform)
            {
                if (t1.name == "HandcraftedSocialSituations")
                {
                    foreach (Transform t2 in t1.gameObject.transform)
                    {
                        if (t2.name == "SpawnLocations")
                        {
                            spawnLocations = t2.gameObject;
                            break;
                        }
                    }
                }
            }
            if (spawnLocations == null)
            {
                throw new System.Exception("Could not find handcrafted game object in social situations game object");
            }
            spawnLocations.SetActive(true);

            agentManager = (Agents.HandCrafted)Agents.BaseAgentManager.instance;
            agentManager.NewScenario(socialSituationEnv, spawnLocations);
        }

        public override Trajectory.TrackedGroup[] groups
        {
            get
            {
                if (agentManager == null)
                {
                    return new Trajectory.TrackedGroup[0];
                }
                return agentManager.groups.ToArray();
            }
        }

        public override Trajectory.TrackedAgent[] agents
        {
            get
            {
                if (agentManager == null)
                {
                    return new Trajectory.TrackedAgent[0];
                }
                return agentManager.agents.ToArray();
            }
        }
    }
}