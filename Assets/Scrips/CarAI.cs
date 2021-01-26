using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using Random = UnityEngine.Random;


namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI : MonoBehaviour
    {
        private CarController m_Car; // the car controller we want to use

        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;

        private void Start()
        {
            // get the car controller
            m_Car = GetComponent<CarController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();

            // Plan your path here
            // Replace the code below that makes a random path
            // ...

            Vector3 start_pos = terrain_manager.myInfo.start_pos;
            Vector3 goal_pos = terrain_manager.myInfo.goal_pos;

            List<Vector3> my_path = new List<Vector3>();


            my_path.Add(start_pos);

            // ----------- RRT* -----------
            // TODO: Use other dimensions of the path tree for higher (or lower) resolution?
            //int[,] path = new int[terrain_manager.myInfo.traversability.GetLength(0), terrain_manager.myInfo.traversability.GetLength(1)];
            PathTree path = new PathTree(nånting nånting start_pos);

            // - Pick a random point a in X.
            // TODO: Probably better to first take out the non-obstacle squares and pick a random from that (to avoid long loop).
            float a = 1;
            while (a != 1)
            {
                int rnd_i = Random.Range(0, terrain_manager.myInfo.traversability.GetLength(0));
                int rnd_j = Random.Range(0, terrain_manager.myInfo.traversability.GetLength(1));
                a = terrain_manager.myInfo.traversability[rnd_i, rnd_j];
            }
            //Debug.Log("i: " + terrain_manager.myInfo.traversability.GetLength(0));
            //Debug.Log("j: " + terrain_manager.myInfo.traversability.GetLength(1));
            //Debug.Log("rnd_i: " + rnd_i);
            //Debug.Log("rnd_j: " + rnd_j);

            // - Find b, the node of the tree closest to a.

            // - Find control inputs u to steer the robot from b to a.

            // - Apply control inputs u for time d, so robot reaches c.

            // - If no collisions occur in getting from a to c:

            //      - Find set of Neighbors N of c.

            //      - Choose Best parent.

            //      - Try to adopt Neighbors (if good).

            // ----------- /RRT* -----------


            //for (int i = 0; i < 3; i++)
            //{
            //    Vector3 waypoint = start_pos;
            //    my_path.Add(waypoint);
            //}
            //my_path.Add(goal_pos);


            // Plot your path to see if it makes sense
            // Note that path can only be seen in "Scene" window, not "Game" window
            Vector3 old_wp = start_pos;
            foreach (var wp in my_path)
            {
                Debug.DrawLine(old_wp, wp, Color.red, 100f);
                old_wp = wp;
            }

            
        }


        private void FixedUpdate()
        {
            // Execute your path here
            // ...

            // this is how you access information about the terrain
            int i = terrain_manager.myInfo.get_i_index(transform.position.x);
            int j = terrain_manager.myInfo.get_j_index(transform.position.z);
            float grid_center_x = terrain_manager.myInfo.get_x_pos(i);
            float grid_center_z = terrain_manager.myInfo.get_z_pos(j);

            Debug.DrawLine(transform.position, new Vector3(grid_center_x, 0f, grid_center_z));


            // this is how you control the car
            m_Car.Move(1f, 1f, 1f, 0f);

        }
    }

    class PathTree<T>
    {
        private T data;
        private LinkedList<PathTree<T>> children;

        public PathTree(T data)
        {
            this.data = data;
            children = new LinkedList<PathTree<T>>();
        }

        public PathTree<T> AddChild(T data)
        {
            PathTree<T> new_child = new PathTree<T>(data);
            children.AddFirst(new_child);
            return new_child;
        }

        public PathTree<T> GetChild(int i)
        {
            foreach (PathTree<T> n in children)
                if (--i == 0)
                    return n;
            return null;
        }

        //public void Traverse(PathTree<T> node, TreeVisitor<T> visitor)
        //{
        //    visitor(node.data);
        //    foreach (PathTree<T> kid in node.children)
        //        Traverse(kid, visitor);
        //}
    }
}
