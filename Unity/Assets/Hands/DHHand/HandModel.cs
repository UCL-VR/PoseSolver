using System.Collections.Generic;
using System.Linq;
using System;
using UCL.CASMS.DH;

namespace UCL.CASMS.Haptics
{
    [Serializable]
    public class Finger : Chain
    {
        public Finger(string name):base(name)
        {
            Joints = new List<Joint>();
            Joints.Add(new Joint("CMCRotation"));
            Joints.Add(new Joint("CMCFlex"));
            Joints.Add(new Joint("MCPAbucation"));
            Joints.Add(new Joint("MCPFlex"));
            Joints.Add(new Joint("PIP"));
            Joints.Add(new Joint("DIP"));
        }

        /// <summary>
        /// The CMC base rotation that accounts for the spread of the metacarpals and twist for palm hollowing
        /// </summary>
        public Joint CMCrotation { get { return Joints[0]; } }

        /// <summary>
        /// The slight inclination of the metacarpals that hollows the palm and offsets the MCP
        /// </summary>
        public Joint CMCflex { get { return Joints[1]; } }

        /// <summary>
        /// MCP Abduction
        /// </summary>
        public Joint MCPabduc { get { return Joints[2]; } }

        /// <summary>
        /// MCP Flex
        /// </summary>
        public Joint MCPflex { get { return Joints[3]; } }

        /// <summary>
        /// PIP
        /// </summary>
        public Joint PIP { get { return Joints[4]; } }

        /// <summary>
        /// DIP
        /// </summary>
        public Joint DIP { get { return Joints[5]; } }
    }

    [Serializable]
    public class Thumb : Chain
    {
        public Thumb():base("Thumb")
        {
            Joints = new List<Joint>();
            Joints.Add(new Joint("CMCProjection"));
            Joints.Add(new Joint("CMCFlex"));
            Joints.Add(new Joint("CMCAbducation"));
            Joints.Add(new Joint("MCPAbduction"));
            Joints.Add(new Joint("MCPFlex"));
            Joints.Add(new Joint("IP"));
        }

        /// <summary>
        /// The linkage from the wrist to the CMC. For the thumb this is a static offset.
        /// </summary>
        public Joint CMCprojection { get { return Joints[0]; } }

        /// <summary>
        /// CMC Flexion
        /// </summary>
        public Joint CMCflex { get { return Joints[1]; } }
        
        /// <summary>
        /// CMC Abducation
        /// </summary>
        public Joint CMCabduc { get { return Joints[2]; } }

        /// <summary>
        /// MCP Abducation
        /// </summary>
        public Joint MCPabduc { get { return Joints[3]; } }

        /// <summary>
        /// MCP Flex
        /// </summary>
        public Joint MCPflex { get { return Joints[4]; } }

        /// <summary>
        /// Proximal Flex
        /// </summary>
        public Joint IP { get { return Joints[5]; } }
    }

    /// <summary>
    /// This class defines a mechanical model for the human hand. The hierarchy defines the kinematic structure and the
    /// constructor defines the actual lengths and rotations.
    /// The kinematic structure 
    /// </summary>
    [Serializable]
    public class HandModel : Model
    {
        float HL;
        float HB;

        public Thumb  Thumb     { get { return Chains[0] as Thumb; } }
        public Finger Index     { get { return Chains[1] as Finger; } }
        public Finger Middle    { get { return Chains[2] as Finger; } }
        public Finger Ring      { get { return Chains[3] as Finger; } }
        public Finger Little    { get { return Chains[4] as Finger; } }

        public HandModel(float hl, float hb) : base("Hand")
        {
            Chains = new List<Chain>();
            Chains.Add(new Thumb());
            Chains.Add(new Finger("Index"));
            Chains.Add(new Finger("Middle"));
            Chains.Add(new Finger("Ring"));
            Chains.Add(new Finger("Little"));
            this.HL = hl;
            this.HB = hb;
        }

        // Default hand paramters. For the dexmo these can be anything so long as they are in the right proportions because finger length doesn't matter.
        public HandModel() : this(0.195f, 0.080f)
        {
            // all the joint flexion, abduction parameters are intialised to zero, even though the resting state of the hand
            // has non-zero parameters.
            // These *do not define the rest state*, they define the *hard constraints* on the linkages between bones. The rest state should be set in the application.

            // For the fingers, we use the kinematic structure similar to that defined by Van Der Hulst et al. for the Ring and Little fingers.
            //
            // The base of each finger starts at the wrist. We use a single DH segment of zero-length for Van Der Hulst's Base Rotation. This rotation spreads
            // the metacarpals and twists the joints so that when they flex they move towards the middle of the palm, hollowing it.
            // The joint is immediately followed by the carpal flex joint, which offsets the MCPs in the direction of the spread and is the bone that actually
            // flexes to hollow the palm.
            // The twists that facilitate palm hollowing are set to zero for the Index and Middle fingers.
            //
            // The MCPs have two DH joints to emulate a saddle joint.
            // The abduction joint offsets the spread to return the fingers to an orientation parallel to the forearm.
            //
            // The DIP, and PIP joints follow as usual.
            //
            // All segment lengths are taken from the model of Buchholz et al.

            Index.CMCrotation.Angle(-6.8f);
            Index.CMCflex.LengthXY(0.447f * HL, 0.251f * HB);
            Index.MCPabduc.Twist(90);
            Index.MCPabduc.Angle(0f - -6.8f);
            Index.MCPflex.Length(0.245f * HL);
            Index.PIP.Length(0.143f * HL);
            Index.DIP.Length(0.097f * HL);

            Middle.CMCrotation.Angle(3.6f);
            Middle.CMCflex.Length(0.446f * HL);
            Middle.MCPabduc.Twist(90);
            Middle.MCPabduc.Angle(2f - 3.6f);
            Middle.MCPflex.Length(0.266f * HL);
            Middle.PIP.Length(0.170f * HL);
            Middle.DIP.Length(0.108f * HL);

            Ring.CMCrotation.Twist(45);
            Ring.CMCrotation.Angle(13.8f);
            Ring.CMCflex.Twist(-45);
            Ring.CMCflex.Length(0.421f * HL);
            Ring.MCPabduc.Twist(90);
            Ring.MCPabduc.Angle(4f - 13.8f);
            Ring.MCPflex.Length(0.244f * HL);
            Ring.PIP.Length(0.165f * HL);
            Ring.DIP.Length(0.107f * HL);
            
            Little.CMCrotation.Twist(45);
            Little.CMCrotation.Angle(23.9f);
            Little.CMCflex.Twist(-45);
            Little.CMCflex.Length(0.414f * HL);
            Little.MCPabduc.Twist(90);
            Little.MCPabduc.Angle(8f - 23.9f);
            Little.MCPflex.Length(0.204f * HL);
            Little.PIP.Length(0.117f * HL);
            Little.DIP.Length(0.093f * HL);

            // For the thumb we use Van Der Hulst et al's model but without the axial rotation of the IP.
            //
            // The projection is a fixed offset set by a single DH node with an offset out and down supported by the translation parameter.
            //
            // The chain contains a saddle joint implemented by two slightly offset DH links, and two flex joint perpendicular to the rest of the
            // hand for the MCP and IP.
            //
            // Again all segment lengths are from Buchholz et al, except the joint center of rotation, whcih is from Van Der Hulst as well.


            Thumb.CMCprojection.Angle(-UnityEngine.Mathf.Atan(UnityEngine.Mathf.Tan(40 * UnityEngine.Mathf.Deg2Rad) * UnityEngine.Mathf.Cos(30 * UnityEngine.Mathf.Deg2Rad)) * UnityEngine.Mathf.Rad2Deg);
            Thumb.CMCprojection.LengthXY(0.073f * HL, -0.196f * HB);
            Thumb.CMCprojection.Pitch(-30);
            Thumb.CMCprojection.Twist(90);
            Thumb.CMCflex.Length(2f * 0.005f);
            Thumb.CMCflex.Twist(-90);
            Thumb.CMCabduc.Length((0.251f * HL) - 0.01f);
            Thumb.CMCabduc.Twist(90f);
            Thumb.MCPabduc.Twist(-90f);
            Thumb.MCPflex.Twist(0);
            Thumb.MCPflex.Length(0.196f * HL);
            Thumb.IP.Length(0.158f * HL);


            // Set the limits from Cobos et al.
            //
            // The direction should be consisent between publications as its coordinate system indepdenent (defined by DH parameters)
            // Negative (min) is Flex and positive (max) is Extension.
            //
            // We have taken the lower bounds on the limits.

            Thumb.CMCflex.RelativeLimits(50, 15);
            Thumb.CMCabduc.RelativeLimits(45, 0);
            Thumb.MCPflex.RelativeLimits(75, 0);
            Thumb.MCPabduc.RelativeLimits(0, 0);
            Thumb.IP.RelativeLimits(75, 5);

            Index.CMCflex.RelativeLimits(0, 0);
            Index.MCPflex.RelativeLimits(90, 10);
            Index.MCPabduc.RelativeLimits(30, 0);
            Index.PIP.RelativeLimits(110, 0);
            Index.DIP.RelativeLimits(80, 5);

            Middle.CMCflex.RelativeLimits(0, 0);
            Middle.MCPflex.RelativeLimits(90, 10);
            Middle.MCPabduc.RelativeLimits(15, 15);
            Middle.PIP.RelativeLimits(110, 0);
            Middle.DIP.RelativeLimits(80, 5);

            Ring.CMCflex.RelativeLimits(10, 0);
            Ring.MCPflex.RelativeLimits(90, 10);
            Ring.MCPabduc.RelativeLimits(0, 30);
            Ring.PIP.RelativeLimits(120, 0);
            Ring.DIP.RelativeLimits(80, 5);

            Little.CMCflex.RelativeLimits(15, 0);
            Little.MCPflex.RelativeLimits(90, 10);
            Little.MCPabduc.RelativeLimits(0, 35);
            Little.PIP.RelativeLimits(135, 0);
            Little.DIP.RelativeLimits(90, 5);
        }
    }
}