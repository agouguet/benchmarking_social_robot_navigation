//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.UnitySim
{
    [Serializable]
    public class TrialStartMsg : Message
    {
        public const string k_RosMessageName = "unity_sim/TrialStart";
        public override string RosMessageName => k_RosMessageName;

        //  Message containing the parameters to start an A-B navigation trial
        public Std.HeaderMsg header;
        public string trial_name;
        //  Which trial name are we running
        public ushort trial_number;
        //  Which trial number are we running
        public Geometry.PoseMsg spawn;
        //  Robot spawn position
        public Geometry.PoseMsg target;
        //  Robot target position
        public Geometry.PoseArrayMsg people;
        //  People spawn positions
        public double time_limit;
        //  Time limit for the trial (in seconds)

        public TrialStartMsg()
        {
            this.header = new Std.HeaderMsg();
            this.trial_name = "";
            this.trial_number = 0;
            this.spawn = new Geometry.PoseMsg();
            this.target = new Geometry.PoseMsg();
            this.people = new Geometry.PoseArrayMsg();
            this.time_limit = 0.0;
        }

        public TrialStartMsg(Std.HeaderMsg header, string trial_name, ushort trial_number, Geometry.PoseMsg spawn, Geometry.PoseMsg target, Geometry.PoseArrayMsg people, double time_limit)
        {
            this.header = header;
            this.trial_name = trial_name;
            this.trial_number = trial_number;
            this.spawn = spawn;
            this.target = target;
            this.people = people;
            this.time_limit = time_limit;
        }

        public static TrialStartMsg Deserialize(MessageDeserializer deserializer) => new TrialStartMsg(deserializer);

        private TrialStartMsg(MessageDeserializer deserializer)
        {
            this.header = Std.HeaderMsg.Deserialize(deserializer);
            deserializer.Read(out this.trial_name);
            deserializer.Read(out this.trial_number);
            this.spawn = Geometry.PoseMsg.Deserialize(deserializer);
            this.target = Geometry.PoseMsg.Deserialize(deserializer);
            this.people = Geometry.PoseArrayMsg.Deserialize(deserializer);
            deserializer.Read(out this.time_limit);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.trial_name);
            serializer.Write(this.trial_number);
            serializer.Write(this.spawn);
            serializer.Write(this.target);
            serializer.Write(this.people);
            serializer.Write(this.time_limit);
        }

        public override string ToString()
        {
            return "TrialStartMsg: " +
            "\nheader: " + header.ToString() +
            "\ntrial_name: " + trial_name.ToString() +
            "\ntrial_number: " + trial_number.ToString() +
            "\nspawn: " + spawn.ToString() +
            "\ntarget: " + target.ToString() +
            "\npeople: " + people.ToString() +
            "\ntime_limit: " + time_limit.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}
