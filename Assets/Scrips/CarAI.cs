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

        private Vector3 Steer(PathTree<Vector3> source, Vector3 target_pos)
        {
            // The Reeds-Shepp/Dubins curves are calculated in the steer function.
            return target_pos;       // Temporarily just return a.
        }

        private bool CheckCollision(PathTree<Vector3> source, Vector3 target_pos)
        {
            // Kolla föreläsning
            return false;
        }

        private void Start()
        {
            // get the car controller
            m_Car = GetComponent<CarController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();

            // Plan your path here

            Vector3 start_pos = terrain_manager.myInfo.start_pos;
            Vector3 goal_pos = terrain_manager.myInfo.goal_pos;

            List<Vector3> my_path = new List<Vector3>();


            my_path.Add(start_pos);


            // ----------- Initializations -----------

            int x_N = terrain_manager.myInfo.x_N;
            int z_N = terrain_manager.myInfo.z_N;
            var traversability = terrain_manager.myInfo.traversability;


            // ----------- RRT* -----------

            int iterations = 100;
            PathTree<Vector3> root = new PathTree<Vector3>(start_pos);

            for (int i = 0; i < iterations; i++)
            {
                // - Pick a random point a in X.
                Vector3 a_pos;
                Vector3 rnd_pos;
                while (true)
                {
                    int x_rnd = Random.Range(0, x_N);
                    int z_rnd = Random.Range(0, z_N);
                    int i_rnd = terrain_manager.myInfo.get_i_index(x_rnd);
                    int j_rnd = terrain_manager.myInfo.get_j_index(z_rnd);
                    rnd_pos = new Vector3(x_rnd, 0.0f, z_rnd);

                    if (traversability[i_rnd, j_rnd] != 0.0f && PathTree<Vector3>.GetNode(rnd_pos) is null)        // Non-obstacle and not already in tree.
                    {
                        a_pos = rnd_pos;
                        break;
                    }
                }

                // - Find b, the node of the tree closest to a.
                PathTree<Vector3> b = root;
                float dist;
                float min_dist = float.PositiveInfinity;
                foreach (KeyValuePair<Vector3, PathTree<Vector3>> kvp in PathTree<Vector3>.node_dict)
                {
                    dist = Vector3.Distance(kvp.Key, a_pos);
                    if (dist < min_dist)
                    {
                        min_dist = dist;
                        b = kvp.Value;
                    }
                }

                // - Find control inputs u to steer the robot from b to a.
                // - Apply control inputs u for time t, so robot reaches c.
                Vector3 c_pos = Steer(b, a_pos);

                // - If no collisions occur in getting from a to c:
                if (CheckCollision(b, c_pos) is false)
                {
                    // For now only RRT:
                    PathTree<Vector3> c = b.AddChild(c_pos);

                    //      - Find set of Neighbors N of c.

                    //      - Choose Best parent.

                    //      - Try to adopt Neighbors (if good).

                    my_path.Add(c_pos);
                }
            }
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
        public static Dictionary<T, PathTree<T>> node_dict = new Dictionary<T, PathTree<T>>();
        private T position;
        private LinkedList<PathTree<T>> children;

        public PathTree(T position)
        {
            this.position = position;
            this.children = new LinkedList<PathTree<T>>();
            node_dict.Add(this.position, this);
        }

        public PathTree<T> AddChild(T position)
        {
            PathTree<T> new_child = new PathTree<T>(position);
            children.AddFirst(new_child);
            return new_child;
        }

        public static PathTree<T> GetNode(T position)
        {
            foreach (KeyValuePair<T, PathTree<T>> kvp in node_dict)
            {
                if (kvp.Key.Equals(position))
                {
                    return kvp.Value;
                }
            }
            return null;
        }
    }


    public class DubinsPath
    {
        // Two circles at the beginning of the path.
        Vector3 start_left_circle;
        Vector3 start_right_circle;
        // Two circles at the end of the path.
        Vector3 goal_left_circle;
        Vector3 goal_right_circle;

        public DubinsPath(Vector3 start_pos, float start_dir, Vector3 goal_pos, float goal_dir)     // Start/goal position and direction of the car.
        {

        }
    }
}
