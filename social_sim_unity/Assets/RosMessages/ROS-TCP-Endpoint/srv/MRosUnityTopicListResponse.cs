//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.ROS_TCP_Endpoint
{
    public class MRosUnityTopicListResponse : Message
    {
        public const string RosMessageName = "ROS-TCP-Endpoint/RosUnityTopicList";

        public string[] topics;

        public MRosUnityTopicListResponse()
        {
            this.topics = new string[0];
        }

        public MRosUnityTopicListResponse(string[] topics)
        {
            this.topics = topics;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();

            listOfSerializations.Add(BitConverter.GetBytes(topics.Length));
            foreach (var entry in topics)
                listOfSerializations.Add(SerializeString(entry));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {

            var topicsArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.topics = new string[topicsArrayLength];
            for (var i = 0; i < topicsArrayLength; i++)
            {
                var topicsStringBytesLength = DeserializeLength(data, offset);
                offset += 4;
                this.topics[i] = DeserializeString(data, offset, topicsStringBytesLength);
                offset += topicsStringBytesLength;
            }

            return offset;
        }

        public override string ToString()
        {
            return "MRosUnityTopicListResponse: " +
            "\ntopics: " + System.String.Join(", ", topics.ToList());
        }
    }
}
