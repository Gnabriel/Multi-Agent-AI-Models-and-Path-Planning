// Disclaimer: We did not write this code.
// Full credit: https://www.habrador.com/tutorials/unity-dubins-paths/3-dubins-paths-in-unity/

using UnityEngine;
using System.Collections;
using System.Collections.Generic;

namespace DubinsPath
{
    //Will hold data related to one Dubins path so we can sort them
    public class OneDubinsPath
    {
        //Tthe total length of this path
        public float totalLength;

        //Need the individual path lengths for debugging and to find the final path
        public float length1;
        public float length2;
        public float length3;

        //The 2 tangent points we need to connect the lines and curves
        public Vector3 tangent1;
        public Vector3 tangent2;

        //The type, such as RSL
        public PathType pathType;

        //The coordinates of the final path
        public List<Vector3> pathCoordinates;

        //To simplify when we generate the final path coordinates
        //Are we turning or driving straight in segment 2?
        public bool segment2Turning;

        //Are we turning right in the particular segment?
        public bool segment1TurningRight;
        public bool segment2TurningRight;
        public bool segment3TurningRight;


        public OneDubinsPath(float length1, float length2, float length3, Vector3 tangent1, Vector3 tangent2, PathType pathType)
        {
            //Calculate the total length of this path
            this.totalLength = length1 + length2 + length3;

            this.length1 = length1;
            this.length2 = length2;
            this.length3 = length3;

            this.tangent1 = tangent1;
            this.tangent2 = tangent2;

            this.pathType = pathType;
        }


        //Are we turning right in any of the segments?
        public void SetIfTurningRight(bool segment1TurningRight, bool segment2TurningRight, bool segment3TurningRight)
        {
            this.segment1TurningRight = segment1TurningRight;
            this.segment2TurningRight = segment2TurningRight;
            this.segment3TurningRight = segment3TurningRight;
        }
    }
}