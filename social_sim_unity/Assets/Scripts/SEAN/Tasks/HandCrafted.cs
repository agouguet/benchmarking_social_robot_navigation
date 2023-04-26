// Copyright (c) 2021, Members of Yale Interactive Machines Group, Yale University,
// Nathan Tsoi
// All rights reserved.
// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree. 

using UnityEngine;

namespace SEAN.Tasks
{
    public class HandCrafted : Base
    {
        protected override bool NewTask()
        {
            Scenario.PedestrianBehavior.HandCrafted scenario = (Scenario.PedestrianBehavior.HandCrafted)sean.pedestrianBehavior;
            // starts the scenario by spawning people and setting their destinations
            scenario.NewScenario();
            return true;
        }

    }
}
