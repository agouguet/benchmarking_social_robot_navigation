// Copyright (c) 2021, Members of Yale Interactive Machines Group, Yale University,
// Nathan Tsoi
// All rights reserved.
// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree. 

using System;
using System.Collections.Generic;
using UnityEngine;

namespace SEAN.Tasks
{
    public class LabStudy : Base
    {
        public List<GameObject> positions = new List<GameObject>();
        public GameObject base_link;
        private int currentPoint;

        public override void Start()
        {
            base.Start();
            currentPoint = 0;

            InitializeTaskPositions();
            onNewTask.Invoke();
            Publish(interactiveGoal);
        }

        protected override void CheckNewTask()
        {
            float distToWayPoint = Vector3.Distance(sean.robot.transform.position, robotGoal.transform.position);
            if (distToWayPoint < completionDistance)
            {
                NewTask();
            }
        }

        private void UpdateWaypoints()
        {
            robotGoal.transform.position = positions[currentPoint + 1].transform.position;
            robotGoal.transform.rotation = positions[currentPoint + 1].transform.rotation;
            playerGoal.transform.position = positions[currentPoint + 1].transform.position;
            playerGoal.transform.rotation = positions[currentPoint + 1].transform.rotation;
            Publish(interactiveGoal);
        }

        private void InitializeTaskPositions()
        {
            foreach (GameObject position in positions)
            {
                position.SetActive(false);
            }
            Vector3 offset1 = new Vector3(-0.75f, 0, 0);
            Vector3 offset2 = new Vector3(0.75f, 0, 0);

            robotStart.transform.position = positions[currentPoint].transform.position;

            robotStart.transform.rotation = positions[currentPoint].transform.rotation;

            playerStart.transform.position = positions[currentPoint].transform.position;
            // increase playerStart y-axis to prevent player from jumping at start of task
            Vector3 startPos = playerStart.transform.position + new Vector3(0, 1, 0);
            playerStart.transform.position = startPos;
            playerStart.transform.rotation = positions[currentPoint].transform.rotation;
            UpdateWaypoints();
        }

        protected override bool NewTask()
        {
            if (currentPoint < positions.Count - 2)
            {
                currentPoint += 1;
                UpdateWaypoints();
                // subtask, not new task
                return false;
            }
            else
            {
                Vector3 v = base_link.GetComponent<Rigidbody>().velocity;
                float vel = (float)System.Math.Round(v.magnitude, 3);
                if(vel == 0){
#if UNITY_EDITOR
                UnityEditor.EditorApplication.isPlaying = false;
#else
                Application.Quit();
#endif
                }
            }
            return true;
        }

    }
}