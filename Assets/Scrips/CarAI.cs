using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using Random = UnityEngine.Random;
using UnityEngine.AI;


namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI : MonoBehaviour
    {
        bool DEBUG_RRT_LIVE = false;
        //int update_count = 0;
        //int overshooter = 0;

        int max_iters = 5000;           // Max iterations or nodes for RRT/RRT*.
        float collision_k = 2.5f;       // Size of the "barrier" for the collision detection (higher values are safer but make it harder to find a path).
        float steer_k = 8f;             // Max distance units for RRT (seems like higher values result in smoother paths).
        //float plan_steer_k = 20f;       // Max distance units for dynamic constraints (if higher than steer_k has no effect; determines detail of final followed path-which is sometimes easier and sometimes harder to actually drive-).
        //float optimal_k = 1f;           // Allowed distance error wrt to RRT for dynamically feasible path (lower values seem to result in more controlled movements).
        //float limitation = 0.4f;        // Fraction of max_spd or acc allowed (right now it only affects speed).
        //float goal_thresh = 0.5f;       // How many units away from the goal can be considered a success.
        

        List<Vector3> next_up = new List<Vector3>();
        LinkedList<List<Vector3>> optimal_path = new LinkedList<List<Vector3>>();
        LinkedList<PathTree> suboptimal_path;


        private CarController m_Car;    // The car controller we want to use.

        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;


        private KinematicCarModel Steer(PathTree source, KinematicCarModel target_state)
        {
            float dt = Time.fixedDeltaTime;

            // Source parameters.
            KinematicCarModel source_state = source.GetState();
            Vector3 source_pos = source_state.GetPosition();
            float source_x = source_pos[0];
            float source_z = source_pos[2];
            float source_orientation = source_state.GetOrientation();

            // Target parameters.
            Vector3 target_pos = target_state.GetPosition();
            float target_x = target_pos[0];
            float target_z = target_pos[2];
            float target_orientation = target_state.GetOrientation();

            float reached_x = (target_x - source_x) / dt;
            float reached_z = (target_z - source_z) / dt;
            Vector3 reached_position = new Vector3(reached_x, 0.0f, reached_z);
            float reached_orientation = (target_orientation - source_orientation) / dt;
            KinematicCarModel reached_state = new KinematicCarModel(reached_position, reached_orientation);

            // This can be utilized somehow to get the input.
            //var input = KinematicCarModel.GetInputFromState(reached_x, reached_z, reached_orientation);
            //float velocity = input.Item1;
            //float steering_angle = input.Item2;

            return reached_state;
        }


        private bool GenerateDubinsPath()
        {
            // Disclaimer: We did not write the code used to generate Dubins Path.
            // Full credit: https://www.habrador.com/tutorials/unity-dubins-paths/3-dubins-paths-in-unity/
            //DubinsPath.DubinsDebug apa = new DubinsPath.DubinsDebug();
            return false;
        }


        private bool CheckCollision(PathTree source, Vector3 target_pos)
        {
            Vector3 current_pos = source.GetPosition();
            Vector3 collision_pos;
            float step = 0.05f * GetDistance(current_pos, target_pos);
            int current_i;
            int current_j;
            float length = collision_k;
            List<Vector3> tilings = new List<Vector3> { Vector3.zero, new Vector3(length, 0, 0), new Vector3(-length, 0, 0), new Vector3(0, 0, length), new Vector3(0, 0, -length),
                new Vector3(length, 0, length), new Vector3(-length, 0, -length), new Vector3(-length, 0, length), new Vector3(length, 0, -length)};

            while (current_pos != target_pos)
            {
                current_pos = Vector3.MoveTowards(current_pos, target_pos, step);
                foreach (Vector3 tile in tilings)
                {
                    collision_pos = current_pos + tile;
                    current_i = terrain_manager.myInfo.get_i_index(collision_pos[0]);
                    current_j = terrain_manager.myInfo.get_j_index(collision_pos[2]);
                    if (terrain_manager.myInfo.traversability[current_i, current_j] == 1)       // Collision.
                    {
                        //Debug.Log("Collision found: " + collision_pos.ToString() + " at i = " + current_i.ToString() + ", j = " + current_j.ToString());
                        return true;
                    }
                }
            }
            return false;                                                                       // No collision.
        }


        private bool __Obsolete__CheckCollision(PathTree source, Vector3 target_pos)
        {
            // Old CheckCollision() without marginalization.
            float step = 1f;        // TODO: Experiment with the step size.
            int current_i;
            int current_j;
            Vector3 current_pos = source.GetPosition();
            while (current_pos != target_pos)
            {
                current_pos = Vector3.MoveTowards(current_pos, target_pos, step);
                current_i = terrain_manager.myInfo.get_i_index(current_pos[0]);
                current_j = terrain_manager.myInfo.get_j_index(current_pos[2]);
                if (terrain_manager.myInfo.traversability[current_i, current_j] == 1)       // Collision.
                {
                    return true;
                }
            }
            return false;                                                                   // No collision.
        }


        private float GetDistance(Vector3 source_pos, Vector3 target_pos)
        {
            // Returns Euclidian distance.
            return Vector3.Distance(source_pos, target_pos);
        }


        private List<PathTree> GetNeighbors(PathTree source)
        {
            float search_radius = 20f;                                              // TODO: Experiment with this value.
            float dist;
            List<PathTree> neighbors = new List<PathTree>();
            foreach (KeyValuePair<Vector3, PathTree> kvp in PathTree.node_dict)
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


        private PathTree RRTStarExpand(Vector3 a_pos, float a_orientation)
        {
            // - Find b, the node of the tree closest to a.
            PathTree b = null;
            float dist;
            float min_dist = float.PositiveInfinity;
            foreach (KeyValuePair<Vector3, PathTree> kvp in PathTree.node_dict)
            {
                dist = GetDistance(kvp.Key, a_pos);
                if (dist < min_dist)
                {
                    min_dist = dist;
                    b = kvp.Value;
                }
            }
            KinematicCarModel a_state = new KinematicCarModel(a_pos, a_orientation);

            // - Find control inputs u to steer the robot from b to a.
            // - Apply control inputs u for time t, so robot reaches c.
            KinematicCarModel c_state = Steer(b, a_state);
            Vector3 c_pos = c_state.GetPosition();
            float c_orientation = c_state.GetOrientation();

            // - If no collisions occur in getting from b to c:
            if (CheckCollision(b, c_pos) is false)
            {
                //      - Add c as child.
                float b_to_c_cost = GetDistance(b.GetPosition(), c_pos);
                PathTree c = b.AddChild(c_pos, c_vel, c_acc, b_to_c_cost);

                //      - Find set of Neighbors N of c.
                List<PathTree> c_neighbors = GetNeighbors(c);

                //      - Choose Best parent.
                PathTree best_parent = b;
                float c_cost_min = c.GetCost();

                foreach (PathTree c_neighbor in c_neighbors)
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
                foreach (PathTree c_neighbor in c_neighbors)
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
            int tree_size = max_iters;
            float root_orientation = 0.0f;                                       // +++++++++++++++++++++++++++++++ TODO: Find a way to get the orientation of start position. +++++++++++++++++++++++++++++++
            PathTree root = new PathTree(start_pos, 0.0f, root_orientation);      // Start with zero velocity.

            //for (int i = 0; i < iterations; i++)
            while (PathTree.node_dict.Count < tree_size)
            {
                // - Pick a random point a in X.
                Vector3 a_pos;
                float a_orientation;
                Vector3 rnd_pos;
                while (true)
                {
                    float x_rnd = Random.Range(x_low, x_high);
                    float z_rnd = Random.Range(z_low, z_high);
                    int i_rnd = terrain_manager.myInfo.get_i_index(x_rnd);
                    int j_rnd = terrain_manager.myInfo.get_j_index(z_rnd);
                    rnd_pos = new Vector3(x_rnd, 0.0f, z_rnd);

                    if (traversability[i_rnd, j_rnd] == 0.0f && PathTree.GetNode(rnd_pos) is null)        // Non-obstacle and not already in tree.
                    {
                        a_pos = rnd_pos;
                        a_orientation = Random.Range(0, 2*Math.PI);
                        break;
                    }
                }
                RRTStarExpand(a_pos, a_orientation);
            }

            // Add the goal to the RRT* path.
            float goal_orientation = -1.0f;                                              // We don't care about the orientation at the goal position.
            PathTree goal = RRTStarExpand(goal_pos, goal_orientation);

            if (goal is null)
            {
                //throw new Exception("ERROR: Goal was not added to the tree. Try searching for more nodes in RRT*.");
                Debug.Log("ERROR: Goal was not added to the tree. Try searching for more nodes in RRT*.");
            }
            // ----------- /RRT* -----------


            // ----------- Draw RRT* Path -----------
            if (DEBUG_RRT_LIVE)
            {
                StartCoroutine(DrawRRTLive(root, goal));                    // Draw the RRT* tree LIVE.
            }
            else
            {
                DrawRRT();                                                  // Draw the whole RRT* tree immediately.
                DrawPath(goal);                                             // Draw the shortest path to goal immediately.
            }
            // ----------- /Draw RRT* Path -----------


            // ----------- Add dynamic constraints to path -----------
            //suboptimal_path = GetPath(goal);
            //PathTree current = suboptimal_path.ElementAt(0);
            //PathTree next = suboptimal_path.ElementAt(1);
            //List<Vector3> reached = new List<Vector3>();
            //reached = SteerLive(current.GetPosition(), next.GetPosition(), current.GetVelocity());
            //List<Vector3> opt_root = new List<Vector3> { current.GetPosition(), current.GetVelocity(), Vector3.zero };
            //optimal_path.AddLast(opt_root);
            //optimal_path.AddLast(reached);
            //for (int k = 1; k < suboptimal_path.Count - 1; k++)
            //{
            //    next = suboptimal_path.ElementAt(k);
            //    while (Vector3.Distance(reached[0], next.GetPosition()) > optimal_k)
            //    {
            //        reached = SteerLive(reached[0], next.GetPosition(), reached[1], true);
            //        optimal_path.AddLast(reached);
            //    }
            //}
            // ----------- /Add dynamic constraints to path -----------


            // ----------- Draw obtained path -----------
            //for (int k = 1; k < optimal_path.Count - 1; k++)
            //{
            //    Debug.DrawLine(optimal_path.ElementAt(k - 1)[0], optimal_path.ElementAt(k)[0], Color.green, 100f);
            //}
            // ----------- /Draw obtained path -----------
        }


        private void FixedUpdate()
        {
            // Execute your path here.
            // This is how you control the car: m_Car.Move(1f, 1f, 1f, 0f);

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


        IEnumerator DrawRRTLive(PathTree root, PathTree goal)
        {
            // Draws the path live with a BFS search.
            WaitForSeconds wait = new WaitForSeconds(0.0001f);                 // Wait time between lines being drawn.
            LinkedList<PathTree> queue = new LinkedList<PathTree>();
            queue.AddLast(root);

            while (true)
            {
                PathTree node = queue.First.Value;
                queue.RemoveFirst();
                foreach (PathTree child in node.GetChildren())
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


        private LinkedList<PathTree> GetPath(PathTree target)
        {
            // Returns a list of nodes where path[0] is start and path[n] is target.
            LinkedList<PathTree> path = new LinkedList<PathTree>();
            PathTree current = target;
            PathTree parent;
            while (!(current.GetParent() is null))
            {
                parent = current.GetParent();
                path.AddFirst(current);
                current = parent;
            }
            return path;
        }


        private void DrawPath(PathTree target)
        {
            LinkedList<PathTree> path = GetPath(target);
            foreach (PathTree node in path)
            {
                if (node.GetPosition() == target.GetPosition())
                {
                    // We have reached the goal.
                    break;
                }
                PathTree parent = node.GetParent();
                Debug.DrawLine(node.GetPosition(), parent.GetPosition(), Color.blue, float.PositiveInfinity);
            }
        }


        IEnumerator DrawPathLive(PathTree target)
        {
            WaitForSeconds wait = new WaitForSeconds(0.2f);                 // Wait time between lines being drawn.
            PathTree current = target;
            PathTree parent;
            while (!(current.GetParent() is null))
            {
                parent = current.GetParent();
                Debug.DrawLine(current.GetPosition(), parent.GetPosition(), Color.blue, float.PositiveInfinity);
                current = parent;
                yield return wait;
            }
        }
    }


    class PathTree
    {
        public static Dictionary<Vector3, PathTree> node_dict = new Dictionary<Vector3, PathTree>();
        private KinematicCarModel state;                                        // Motion model of the vehicle.
        private float velocity;
        private float cost;                                     // Total cost to reach this node.
        private LinkedList<PathTree> children;               // List of this nodes' children.
        private PathTree parent;                             // This nodes' parent.

        public PathTree(Vector3 position, float velocity, float orientation)
        {
            // Constructor for root node.
            this.state = new KinematicCarModel(position, orientation);
            this.velocity = velocity;
            this.cost = 0f;
            this.children = new LinkedList<PathTree>();
            this.parent = null;
            node_dict.Add(this.state.GetPosition(), this);
        }

        public PathTree(Vector3 position, float velocity, float orientation, float cost)
        {
            // Constructor for non-root nodes.
            this.state = new KinematicCarModel(position, orientation);
            this.velocity = velocity;
            this.cost = cost;
            this.children = new LinkedList<PathTree>();
            node_dict.Add(this.state.GetPosition(), this);
        }

        public PathTree AddChild(Vector3 position, float velocity, float orientation, float sub_cost)
        {
            float child_cost = this.cost + sub_cost;
            PathTree new_child = new PathTree(position, velocity, orientation, child_cost);
            this.children.AddLast(new_child);
            new_child.parent = this;
            return new_child;
        }

        public bool RemoveChild(PathTree child)
        {
            return children.Remove(child);
        }

        public PathTree AdoptChild(PathTree child, float child_cost)
        {
            child.parent.RemoveChild(child);                        // Removes old parent.
            this.children.AddLast(child);                           // Adds the child among this' children.
            child.parent = this;                                    // Adds this as new parent.
            child.cost = child_cost;                                // Updates cost of child according to the distance through this parent.
            return child;
        }

        public Vector3 GetPosition()
        {
            return this.state.GetPosition();
        }

        public float GetVelocity()
        {
            return this.velocity;
        }

        public float GetCost()
        {
            return this.cost;
        }

        public KinematicCarModel GetState()
        {
            return this.state;
        }

        public PathTree GetParent()
        {
            return this.parent;
        }

        public LinkedList<PathTree> GetChildren()
        {
            return this.children;
        }

        public static PathTree GetNode(Vector3 position)
        {
            foreach (KeyValuePair<Vector3, PathTree> kvp in node_dict)
            {
                if (kvp.Key.Equals(position))
                {
                    return kvp.Value;
                }
            }
            return null;
        }
    }


    class KinematicCarModel
    {
        // A class that represents the state of a vehicle according to the Kinematic Car Model.
        private Vector3 position;                                   // Position of center of gravity.
        private float theta;                                        // The orientation of the vehicle (in radians).
        
        private static float length = 5.0f;                         // Vehicle length.  (from Group 3)
        private static float width = 2.5f;                          // Vehicle width.   (from Group 3)
        private static float v_max = float.PositiveInfinity;        // Maximum velocity.
        private static float phi_max = float.PositiveInfinity;      // Maximum steering angle.

        public KinematicCarModel(Vector3 position, float orientation)
        {
            this.position = position;
            this.theta = orientation;
        }

        public void UpdateState(float v, float phi)
        {
            // v: Forward velocity.
            // phi: Steering angle (in radians).

            // Make sure we do not steer more than the max steering angle at neither left or right.
            if (phi > phi_max)
            {
                phi = phi_max;
            }
            else if (phi < -phi_max)
            {
                phi = -phi_max;
            }
            // Limit velocity to max velocity.
            v = Math.Max(v, v_max);

            float x_pos = v * (float)Math.Cos(phi);
            float z_pos = v * (float)Math.Sin(phi);
            this.position = new Vector3(x_pos, 0.0f, z_pos);
            this.theta = (v / length) * (float)Math.Tan(phi);
        }

        public Vector3 GetPosition()
        {
            return this.position;
        }

        public float GetOrientation()
        {
            return this.theta;
        }

        public static (float, float) GetInputFromState(float x_dot, float z_dot, float theta_dot)
        {
            // Derive theta 0:
            // v = x / cos(0) = y / sin(0)
            // y = x * sin(0) / cos(0) = x * tan(0)
            // tan(0) = y / x
            // 0 = arctan(y / x)
            float theta = (float)Math.Atan(x_dot / z_dot);
            float v = x_dot / (float)Math.Cos(theta);
            float phi = (float)Math.Atan((theta_dot * length) / v);
            return (v, phi);
        }

        public static float NormalizeAngle(float angle)
        {
            // Takes an angle and normalizes it to [-pi, pi].
            // Not sure if we will need this.
            float pi = (float)Math.PI;
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
            float x_pos_new = x_pos + (this.v_x * (float)Math.Cos(this.r) * dt) - (this.v_y * (float)Math.Sin(this.r) * dt);
            float y_pos_new = y_pos + (this.v_x * (float)Math.Sin(this.r) * dt) + (this.v_y * (float)Math.Cos(this.r) * dt);

            // New orientation.
            float omega_new = this.r;

            // New y-velocity.
            float v_y_new = (F_yf / m) * (float)Math.Cos(delta_f) + (F_yr / m) - this.v_x * this.r;

            // New turn rate.
            float r_new = (L_f / I_z) * F_yf * (float)Math.Cos(delta_f) - (L_r / I_z) * F_yr;

            //float r_new = DynamicBicycleModel.NormalizeAngle(this.r + this.omega * dt);           // From: github.com/Derekabc.

            // Update state.
            this.position = new Vector3(x_pos_new, 0.0f, y_pos_new);
            this.omega = omega_new;
            this.v_x = 0.0f;                                                                        // ?? what should this be??
            this.v_y = v_y_new;
            this.r = r_new;


        }

        public static float NormalizeAngle(float angle)
        {
            // Takes an angle and normalizes it to [-pi, pi].
            float pi = (float)Math.PI;
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
