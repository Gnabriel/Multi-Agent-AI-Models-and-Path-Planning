// Disclaimer: We did not write this code.
// Full credit: https://www.habrador.com/tutorials/unity-dubins-paths/3-dubins-paths-in-unity/

using UnityEngine;
using System.Collections;
using System.Collections.Generic;

namespace DubinsPath
{
    //Display the final Dubins Paths
    public class DubinsDebug : MonoBehaviour
    {
        //Everything we need to add in the editor
        //Circles
        public Transform goalCircleLeft;
        public Transform goalCircleRight;
        public Transform startCircleLeft;
        public Transform startCircleRight;
        //Line renderers
        public LineRenderer lineRSR;
        public LineRenderer lineLSL;
        public LineRenderer lineRSL;
        public LineRenderer lineLSR;
        public LineRenderer lineRLR;
        public LineRenderer lineLRL;
        //The cars we generate paths to/from
        public Transform startCar;
        public Transform goalCar;

        //Objects
        DubinsGeneratePaths dubinsPathGenerator;


        void Start()
        {
            dubinsPathGenerator = new DubinsGeneratePaths();
        }


        void Update()
        {
            //To generate paths we need the position and rotation (heading) of the cars
            Vector3 startPos = startCar.position;
            Vector3 goalPos = goalCar.position;
            //Heading is in radians
            float startHeading = startCar.eulerAngles.y * Mathf.Deg2Rad;
            float goalHeading = goalCar.eulerAngles.y * Mathf.Deg2Rad;

            //Get all valid Dubins paths
            List<OneDubinsPath> pathDataList = dubinsPathGenerator.GetAllDubinsPaths(
                startPos,
                startHeading,
                goalPos,
                goalHeading);

            //If we have paths
            if (pathDataList.Count > 0)
            {
                //Display all paths with line renderers
                DebugAllPaths(pathDataList);
            }

            //Position the left and right circles
            PositionLeftRightCircle();
        }


        //Position the left and right circle objects for debugging
        void PositionLeftRightCircle()
        {
            goalCircleLeft.position = dubinsPathGenerator.goalLeftCircle;
            goalCircleRight.position = dubinsPathGenerator.goalRightCircle;

            startCircleLeft.position = dubinsPathGenerator.startLeftCircle;
            startCircleRight.position = dubinsPathGenerator.startRightCircle;
        }


        //Displaying all Dubins paths with line renderers for debugging
        void DebugAllPaths(List<OneDubinsPath> pathDataList)
        {
            //Deactivate all line renderers (we activate them if a path is available)
            DeactivateLineRenderers();

            for (int i = 0; i < pathDataList.Count; i++)
            {
                PathType currentPathType = pathDataList[i].pathType;

                switch (currentPathType)
                {
                    case PathType.LRL:
                        DisplayPath(pathDataList[i], lineLRL);
                        break;
                    case PathType.RLR:
                        DisplayPath(pathDataList[i], lineRLR);
                        break;
                    case PathType.LSR:
                        DisplayPath(pathDataList[i], lineLSR);
                        break;
                    case PathType.RSL:
                        DisplayPath(pathDataList[i], lineRSL);
                        break;
                    case PathType.RSR:
                        DisplayPath(pathDataList[i], lineRSR);
                        break;
                    case PathType.LSL:
                        DisplayPath(pathDataList[i], lineLSL);
                        break;
                }
            }
        }


        //Display a path with a line renderer
        void DisplayPath(OneDubinsPath pathData, LineRenderer lineRenderer)
        {
            //Activate the line renderer
            lineRenderer.gameObject.SetActive(true);

            //The coordinates of the path
            List<Vector3> pathCoordinates = pathData.pathCoordinates;

            //Display the final line
            lineRenderer.positionCount = pathCoordinates.Count;

            for (int i = 0; i < pathCoordinates.Count; i++)
            {
                lineRenderer.SetPosition(i, pathCoordinates[i]);
            }
        }


        //Deactivate all line renderers in case a circle is not possible
        //Then we dont want to show the old circle
        void DeactivateLineRenderers()
        {
            lineLRL.gameObject.SetActive(false);
            lineRLR.gameObject.SetActive(false);
            lineLSL.gameObject.SetActive(false);
            lineRSR.gameObject.SetActive(false);
            lineLSR.gameObject.SetActive(false);
            lineRSL.gameObject.SetActive(false);
        }
    }
}