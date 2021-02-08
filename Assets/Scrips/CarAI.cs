﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using Random = UnityEngine.Random;


namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI : MonoBehaviour
    {
        bool DEBUG_RRT_LIVE = false;

        private CarController m_Car; // the car controller we want to use

        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;

        private Vector3 Steer(PathTree<Vector3> source, Vector3 target_pos)
        {
            // The Reeds-Shepp/Dubins curves are calculated in the steer function.
            return target_pos;       // Temporarily just return a.
        }

        private bool GenerateDubinsPath()
        {
            // Disclaimer: We did not write the code used to generate Dubins Path.
            // Full credit: https://www.habrador.com/tutorials/unity-dubins-paths/3-dubins-paths-in-unity/
            //DubinsPath.DubinsDebug apa = new DubinsPath.DubinsDebug();
            return false;
        }

        private bool CheckCollision(PathTree<Vector3> source, Vector3 target_pos)
        {
            float step = 1f;        // TODO: Experiment with the step size.
            int current_i;
            int current_j;
            Vector3 current_pos = source.GetPosition();

            //Debug.Log("---------");
            //Debug.Log("Source position: " + current_pos.ToString());
            //Debug.Log("Target position: " + target_pos.ToString());

            while (current_pos != target_pos)
            {
                current_pos = Vector3.MoveTowards(current_pos, target_pos, step);
                current_i = terrain_manager.myInfo.get_i_index(current_pos[0]);
                current_j = terrain_manager.myInfo.get_j_index(current_pos[2]);

                //Debug.Log("Current position: " + current_pos.ToString());
                //Debug.Log("Current i = " + current_i.ToString() + ", j = " + current_j.ToString());

                if (terrain_manager.myInfo.traversability[current_i, current_j] == 1)       // Collision.
                {
                    //Debug.Log("Collision found: " + current_pos.ToString() + " at i = " + current_i.ToString() + ", j = " + current_j.ToString());
                    return true;
                }
            }

            //Debug.Log("No collision found. This path is clear.");
            //Debug.Log("---------");
            //Debug.Log("");
            return false;                                                           // No collision.
        }

        private float GetDistance(Vector3 source_pos, Vector3 target_pos)
        {
            return Vector3.Distance(source_pos, target_pos);
        }

        private List<PathTree<Vector3>> GetNeighbors(PathTree<Vector3> source)
        {
            float search_radius = 20f;                                              // TODO: Experiment with this value.
            float dist;
            List<PathTree<Vector3>> neighbors = new List<PathTree<Vector3>>();
            foreach (KeyValuePair<Vector3, PathTree<Vector3>> kvp in PathTree<Vector3>.node_dict)
            {
                dist = GetDistance(kvp.Key, source.GetPosition());
                if (dist <= search_radius)
                {
                    neighbors.Add(kvp.Value);
                }
            }
            if (neighbors.Count == 0)
            {
                return null;
            }
            return neighbors;
        }

        private PathTree<Vector3> RRTStarExpand(Vector3 a_pos)
        {
            // - Find b, the node of the tree closest to a.
            PathTree<Vector3> b = null;
            float dist;
            float min_dist = float.PositiveInfinity;
            foreach (KeyValuePair<Vector3, PathTree<Vector3>> kvp in PathTree<Vector3>.node_dict)
            {
                dist = GetDistance(kvp.Key, a_pos);
                if (dist < min_dist)
                {
                    min_dist = dist;
                    b = kvp.Value;
                }
            }

            // - Find control inputs u to steer the robot from b to a.
            // - Apply control inputs u for time t, so robot reaches c.
            Vector3 c_pos = Steer(b, a_pos);

            // - If no collisions occur in getting from b to c:
            if (CheckCollision(b, c_pos) is false)
            {
                //      - Add c as child.
                float b_to_c_cost = GetDistance(b.GetPosition(), c_pos);
                PathTree<Vector3> c = b.AddChild(c_pos, b_to_c_cost);

                //      - Find set of Neighbors N of c.
                List<PathTree<Vector3>> c_neighbors = GetNeighbors(c);

                //      - Choose Best parent.
                PathTree<Vector3> best_parent = b;
                float c_cost_min = c.GetCost();

                foreach (PathTree<Vector3> c_neighbor in c_neighbors)
                {
                    float neighbor_to_c_cost = GetDistance(c_neighbor.GetPosition(), c.GetPosition());
                    float c_cost_new = c_neighbor.GetCost() + neighbor_to_c_cost;
                    if (CheckCollision(c_neighbor, c.GetPosition()) is false && c_cost_new < c_cost_min)
                    {
                        best_parent = c_neighbor;
                        c_cost_min = c_cost_new;
                    }
                }
                best_parent.AdoptChild(c, c_cost_min);

                //      - Try to adopt Neighbors (if good).
                foreach (PathTree<Vector3> c_neighbor in c_neighbors)
                {
                    float c_to_neighbor_cost = GetDistance(c.GetPosition(), c_neighbor.GetPosition());
                    float neighbor_cost_new = c.GetCost() + c_to_neighbor_cost;
                    if (CheckCollision(c, c_neighbor.GetPosition()) is false && neighbor_cost_new < c_neighbor.GetCost())
                    {
                        c.AdoptChild(c_neighbor, neighbor_cost_new);
                    }
                }
                return c;
            }
            return null;
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

            float x_low = terrain_manager.myInfo.x_low;
            float x_high = terrain_manager.myInfo.x_high;
            float z_low = terrain_manager.myInfo.z_low;
            float z_high = terrain_manager.myInfo.z_high;
            var traversability = terrain_manager.myInfo.traversability;

            // ----------- /Initializations -----------


            // ----------- RRT* -----------

            //int iterations = 5000;
            int tree_size = 10000;
            PathTree<Vector3> root = new PathTree<Vector3>(start_pos);

            //for (int i = 0; i < iterations; i++)
            while (PathTree<Vector3>.node_dict.Count < tree_size)
            {
                // - Pick a random point a in X.
                Vector3 a_pos;
                Vector3 rnd_pos;
                while (true)
                {
                    float x_rnd = Random.Range(x_low, x_high);
                    float z_rnd = Random.Range(z_low, z_high);
                    int i_rnd = terrain_manager.myInfo.get_i_index(x_rnd);
                    int j_rnd = terrain_manager.myInfo.get_j_index(z_rnd);
                    rnd_pos = new Vector3(x_rnd, 0.0f, z_rnd);

                    if (traversability[i_rnd, j_rnd] == 0.0f && PathTree<Vector3>.GetNode(rnd_pos) is null)        // Non-obstacle and not already in tree.
                    {
                        a_pos = rnd_pos;
                        break;
                    }
                }

                RRTStarExpand(a_pos);
            }

            // Add the goal to the RRT* path.
            PathTree<Vector3> goal = RRTStarExpand(goal_pos);

            if (goal is null)
            {
                throw new Exception("ERROR: Goal was not added to the tree. Try searching for more nodes in RRT*.");
            }

            // ----------- /RRT* -----------


            // ----------- Draw RRT* Path -----------

            if (DEBUG_RRT_LIVE)
            {
                StartCoroutine(DrawRRTLive(root, goal));                  // Draw the RRT* tree LIVE.
            }
            else
            {
                DrawRRT();                                          // Draw the whole RRT* tree immediately.
                DrawPath(goal);                                     // Draw the shortest path to goal immediately.
            }

            // ----------- /Draw RRT* Path -----------


            //for (int i = 0; i < 3; i++)
            //{
            //    Vector3 waypoint = start_pos;
            //    my_path.Add(waypoint);
            //}
            //my_path.Add(goal_pos);
        }


        private void FixedUpdate()
        {
            // Execute your path here
            // ...

            // this is how you access information about the terrain
            //int i = terrain_manager.myInfo.get_i_index(transform.position.x);
            //int j = terrain_manager.myInfo.get_j_index(transform.position.z);
            //float grid_center_x = terrain_manager.myInfo.get_x_pos(i);
            //float grid_center_z = terrain_manager.myInfo.get_z_pos(j);

            //Debug.DrawLine(transform.position, new Vector3(grid_center_x, 0f, grid_center_z));


            // this is how you control the car
            m_Car.Move(1f, 1f, 1f, 0f);

        }


        private void DrawRRT()
        {
            foreach (KeyValuePair<Vector3, PathTree<Vector3>> kvp in PathTree<Vector3>.node_dict)
            {
                PathTree<Vector3> node = kvp.Value;
                foreach (PathTree<Vector3> child in node.GetChildren())
                {
                    Debug.DrawLine(node.GetPosition(), child.GetPosition(), Color.red, float.PositiveInfinity);
                }
            }
        }


        IEnumerator DrawRRTLive(PathTree<Vector3> root, PathTree<Vector3> goal)
        {
            // Draws the path live with a BFS search.
            WaitForSeconds wait = new WaitForSeconds(0.0001f);                 // Wait time between lines being drawn.
            LinkedList<PathTree<Vector3>> queue = new LinkedList<PathTree<Vector3>>();
            queue.AddLast(root);

            while (true)
            {
                PathTree<Vector3> node = queue.First.Value;
                queue.RemoveFirst();
                foreach (PathTree<Vector3> child in node.GetChildren())
                {
                    queue.AddLast(child);
                    Debug.DrawLine(node.GetPosition(), child.GetPosition(), Color.red, float.PositiveInfinity);
                    yield return wait;
                }
                if (queue.Last is null)
                {
                    break;
                }
            }
            StartCoroutine(DrawPathLive(goal));                 // Draw the shortest path to goal LIVE.
        }


        private LinkedList<PathTree<Vector3>> GetPath(PathTree<Vector3> target)
        {
            // Returns a list of nodes where path[0] is start and path[n] is target.
            LinkedList<PathTree<Vector3>> path = new LinkedList<PathTree<Vector3>>();
            PathTree<Vector3> current = target;
            PathTree<Vector3> parent;
            while (!(current.GetParent() is null))
            {
                parent = current.GetParent();
                path.AddFirst(current);
                current = parent;
            }
            return path;
        }


        private void DrawPath(PathTree<Vector3> target)
        {
            LinkedList<PathTree<Vector3>> path = GetPath(target);
            foreach (PathTree<Vector3> node in path)
            {
                if (node.GetPosition() == target.GetPosition())
                {
                    // We have reached the goal.
                    break;
                }
                PathTree<Vector3> parent = node.GetParent();
                Debug.DrawLine(node.GetPosition(), parent.GetPosition(), Color.blue, float.PositiveInfinity);
            }
        }


        IEnumerator DrawPathLive(PathTree<Vector3> target)
        {
            WaitForSeconds wait = new WaitForSeconds(0.2f);                 // Wait time between lines being drawn.
            PathTree<Vector3> current = target;
            PathTree<Vector3> parent;
            while (!(current.GetParent() is null))
            {
                parent = current.GetParent();
                Debug.DrawLine(current.GetPosition(), parent.GetPosition(), Color.blue, float.PositiveInfinity);
                current = parent;
                yield return wait;
            }
        }
    }


    class PathTree<T>
    {
        public static Dictionary<T, PathTree<T>> node_dict = new Dictionary<T, PathTree<T>>();
        private T position;                                     // Position of this node.
        private float cost;                                     // Total cost to reach this node.
        private LinkedList<PathTree<T>> children;               // List of this nodes' children.
        private PathTree<T> parent;                             // This nodes' parent.

        public PathTree(T position)
        {
            // Constructor for root node.
            this.position = position;
            this.cost = 0f;
            this.children = new LinkedList<PathTree<T>>();
            this.parent = null;
            node_dict.Add(this.position, this);
        }

        public PathTree(T position, float cost)
        {
            // Constructor for non-root nodes.
            this.position = position;
            this.cost = cost;
            this.children = new LinkedList<PathTree<T>>();
            node_dict.Add(this.position, this);
        }

        public PathTree<T> AddChild(T position, float sub_cost)
        {
            float child_cost = this.cost + sub_cost;
            PathTree<T> new_child = new PathTree<T>(position, child_cost);
            this.children.AddLast(new_child);
            new_child.parent = this;
            return new_child;
        }

        public bool RemoveChild(PathTree<T> child)
        {
            return children.Remove(child);
        }

        public PathTree<T> AdoptChild(PathTree<T> child, float child_cost)
        {
            child.parent.RemoveChild(child);                        // Removes old parent.
            this.children.AddLast(child);                           // Adds the child among this' children.
            child.parent = this;                                    // Adds this as new parent.
            child.cost = child_cost;                                // Updates cost of child according to the distance through this parent.
            return child;
        }

        public T GetPosition()
        {
            return this.position;
        }

        public float GetCost()
        {
            return this.cost;
        }

        public PathTree<T> GetParent()
        {
            return this.parent;
        }

        public LinkedList<PathTree<T>> GetChildren()
        {
            return this.children;
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


    class DynamicBicycleModel
    {
        // A class that represents the state of a vehicle according to the Dynamic Bicycle Model.
        private Vector3 position;                               // Position of center of gravity.
        private float omega;                                    // The orientation of the vehicle (in radians).
        private float v_x;                                      // Forward speed.
        private float v_y;                                      // Lateral speed.
        private float r;                                        // Yaw (turn) rate.

        private static float dt = Time.fixedDeltaTime;
        // TODO: Change placeholder values below.
        private static float L = 2.0f;                          // Vehicle length in [m].
        private static float L_f = L / 2;                       // Vehicle front length in [m] (from center of gravity).
        private static float L_r = L / 2;                       // Vehicle rear length in [m] (from center of gravity).
        private static float m = 2000f;                         // Vehicle weight in [kg].


        public DynamicBicycleModel(Vector3 position, float orientation, float x_velocity, float y_velocity, float yaw_rate)
        {
            this.position = position;
            this.omega = orientation;
            this.v_x = x_velocity;
            this.v_y = y_velocity;
            this.r = yaw_rate;
        }

        public void UpdateState(float max_steering,  float acceleration, float steering)
        {
            // Make sure we do not steer more than the max steering angle at neither left or right.
            if (steering > max_steering)
            {
                steering = max_steering;
            }
            else if (steering < -max_steering)
            {
                steering = -max_steering;
            }

            float delta_f = steering;       // TODO: Is this correct?

            // TODO: Initialize these.
            float C_alphaf = 0.0f;
            float C_alphar = 0.0f;

            // "A vehicle with a low polar moment of inertia gives a quick response to steering commands."
            // "A vehicle with a high polar moment has high directional stability (meaning it resists changing its direction)."
            float I_z = 0.0f;                 // Should we decide this with trial and error?

            float alpha_f = (this.v_y + L_f * this.r) / this.v_x + delta_f;
            float alpha_r = (this.v_y - L_r * this.r) / this.v_x;

            float F_yf = -C_alphaf * alpha_f;
            float F_yr = -C_alphar * alpha_r;




            // Current position.
            float x_pos = this.position[0];
            float y_pos = this.position[2];

            // New position.
            float x_pos_new = x_pos + (this.v_x * Math.Cos(this.r) * dt) - (this.v_y * Math.Sin(this.r) * dt);
            float y_pos_new = y_pos + (this.v_x * Math.Sin(this.r) * dt) + (this.v_y * Math.Cos(this.r) * dt);

            // New orientation.
            float omega_new = this.r;

            // New y-velocity.
            float v_y_new = (F_yf / m) * Math.Cos(delta_f) + (F_yr / m) - this.v_x * this.r;

            // New turn rate.
            float r_new = (L_f / I_z) * F_yf * Math.Cos(delta_f) - (L_r / I_z) * F_yr;

            //float r_new = DynamicBicycleModel.NormalizeAngle(this.r + this.omega * dt);           // From: github.com/Derekabc.

            // Update state.
            this.position = new Vector3(x_pos_new, 0.0f, y_pos_new);
            this.omega = omega_new;
            this.v_x = null;                                                                        // ?? what should this be??
            this.v_y = v_y_new;
            this.r = r_new;


        }

        public static float NormalizeAngle(float angle)
        {
            // Takes an angle and normalizes it to [-pi, pi].
            double pi = Math.PI;
            while (angle > pi)
            {
                angle -= 2 * pi;
            }
            while (angle < -pi)
            {
                angle += 2 * pi;
            }
            float normalized_angle = angle;
            return normalized_angle;
        }
    }
}
