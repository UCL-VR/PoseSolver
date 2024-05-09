using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UCL.CASMS.DH;
using Joint = UCL.CASMS.DH.Joint;

public class DHHandHelper : MonoBehaviour
{
    public class Joints
    {
        public string Name;
        public List<Joint> joints;
    }

    public class Finger : Joints
    {
        public Finger()
        {
            joints = new List<Joint>();
        }

        public Finger(Transform t)
        {
            joints = t.GetComponentsInChildren<DHJointLink>().Select(dh => dh.joint).ToList();
            Name = t.name;
        }

        /// <summary>
        /// The angle to the left or right of the middle finger (straight ahead) pointing towards the knuckle
        /// </summary>
        public Joint CMCrotation => joints[0];

        /// <summary>
        /// The slight inclination of the metacarpals that hollows the palm and offsets the MCP
        /// </summary>
        public Joint CMCflex => joints[1];

        /// <summary>
        /// The side to side rotation of the finger around the knuckle
        /// </summary>
        public Joint MCPabduc => joints[2];

        /// <summary>
        /// The flex (i.e. up and down rotation) of the finger around the knuckle (power grasp pose)
        /// </summary>
        public Joint MCPflex => joints[3];

        /// <summary>
        /// The flex of the proximal interphalangeal joint (the first knuckle)
        /// </summary>
        public Joint PIP => joints[4];

        /// <summary>
        /// The flex of the distal interphalangeal joint (the second knuckle)
        /// </summary>
        public Joint DIP => joints[5];
    }

    public class Thumb : Joints
    {
        public Thumb()
        {
            joints = new List<Joint>();
        }

        public Thumb(Transform t)
        {
            joints = t.GetComponentsInChildren<DHJointLink>().Select(dh => dh.joint).ToList();
            Name = t.name;
        }

        /// <summary>
        /// The linkage from the wrist to the CMC. For the thumb this is a static offset.
        /// </summary>
        public Joint CMCprojection => joints[0]; 

        /// <summary>
        /// CMC Flexion
        /// </summary>
        public Joint CMCflex => joints[1]; 

        /// <summary>
        /// CMC Abducation
        /// </summary>
        public Joint CMCabduc => joints[2]; 

        /// <summary>
        /// MCP Abducation
        /// </summary>
        public Joint MCPabduc => joints[3]; 

        /// <summary>
        /// MCP Flex
        /// </summary>
        public Joint MCPflex => joints[4]; 

        /// <summary>
        /// Proximal Flex
        /// </summary>
        public Joint IP => joints[5]; 
    }


    public class Model
    {
        public Finger Index;
        public Finger Middle;
        public Finger Ring;
        public Finger Little;
        public Thumb Thumb;
    }

    public Model GetModel()
    {
        var model = new Model();
        model.Index = new Finger(transform.Find("Index"));
        model.Middle = new Finger(transform.Find("Middle"));
        model.Ring = new Finger(transform.Find("Ring"));
        model.Little = new Finger(transform.Find("Little"));
        model.Thumb = new Thumb(transform.Find("Thumb"));
        return model;
    }
}
