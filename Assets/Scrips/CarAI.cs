using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using Random = UnityEngine.Random;
using System.Linq;
using UnityEngine.AI;


namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI : MonoBehaviour
    {
        bool DEBUG_RRT_LIVE = false;
        //int overshooter = 0;
        int update_count = 0;
        int max_iters = 25000;           // Max iterations or nodes for RRT/RRT*.
        float collision_k = 3.5f;       // Size of the "barrier" for the collision detection (higher values are safer but make it harder to find a path).
        float steer_k = 3f;             // Max distance units for RRT (seems like higher values result in smoother paths).
        float plan_steer_k = 20f;       // Max distance units for dynamic constraints (if higher than steer_k has no effect; determines detail of final followed path-which is sometimes easier and sometimes harder to actually drive-).
        float optimal_k = 3f;           // Allowed distance error wrt to RRT for dynamically feasible path (lower values seem to result in more controlled movements).
        float limitation = 0.3f;        // Fraction of max_spd or acc allowed (right now it only affects speed).
        float goal_thresh = 0.5f;       // How many units away from the goal can be considered a success.
        int pwm_steady = 12;
        int pwm_turn = 31;
        int pwm_count = 0;
        int pwm;
        bool pwm_switch = false;
        float global_ori = (float)Math.PI*0.5f;
        float prev_ori = (float)Math.PI * 0.5f;
        Vector3 acc;
        float length = 4.0f;


        List<Vector3> next_up = new List<Vector3>();
        LinkedList<List<Vector3>> optimal_path = new LinkedList<List<Vector3>>();
        LinkedList<PathTree> suboptimal_path = new LinkedList<PathTree>();
        Vector3 MyVelocity = Vector3.zero;
        Vector3 MyOldPos;

        PathTree current_car;
        PathTree next_car;


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

            var input = KinematicCarModel.GetInputFromState(reached_x, reached_z, reached_orientation);
            float velocity = input.Item1;
            //float steering_angle = input.Item2;
            KinematicCarModel reached_state = new KinematicCarModel(reached_position, reached_orientation, velocity);
            return reached_state;
        }

        private Vector3 EasySteer(PathTree source, Vector3 target_pos)
        {
            float pi = (float)Math.PI;
            float max_steer = (2.5f * m_Car.m_MaximumSteerAngle * pi) / 180f;
            float turn;
            float len;
            Vector3 current_pos = source.GetPosition();
            Vector3 reached_pos = Vector3.zero;
            /*
            float ang_dif = (float)Math.Atan2((target_pos - current_pos).z, (target_pos - current_pos).x);
            if ((ang_dif > pi*0.5f + max_steer) || (ang_dif < -pi * 0.5f))
            {
                turn = pi * 0.5f + max_steer;
            }else
            {
                if(ang_dif < pi * 0.5f - max_steer)
                {
                    turn = pi * 0.5f - max_steer;
                } else
                {
                    turn = 0f;
                }
            }
            if(turn != 0f)
            {
                //target_pos = new Vector3((float)Math.Cos(turn) + current_pos.x, 0f, (float)Math.Cos(turn) + current_pos.z);
                return current_pos;
            }
            */
            len = Math.Min(Vector3.Distance(current_pos, target_pos), steer_k);
            reached_pos = Vector3.MoveTowards(current_pos, target_pos, len);//modest goal
            return reached_pos;
        }

        private List<Vector3> SteerLive(Vector3 from_v, Vector3 to_v, Vector3 from_vel, float from_orientation = 0f, bool plan = false)
        {
            if (plan && Vector3.Distance(from_v, to_v) > plan_steer_k)
            {
                to_v = Vector3.MoveTowards(from_v, to_v, plan_steer_k);//modest goal
            }
            //Debug.Log("Steeeer");
            float dt = Time.fixedDeltaTime;
            float max_spd = m_Car.MaxSpeed * limitation;
            if ((Vector3.Distance(from_v, to_v) >= 0.9f * plan_steer_k) && false)
            {
                max_spd += m_Car.MaxSpeed * ((1f - limitation) / 2f);
            }
            float max_acc = max_spd;
            Vector3 target_vel = (to_v - from_v) / dt;
            Vector3 true_vel;
            if (target_vel.magnitude > max_spd)
            {
                target_vel = target_vel.normalized * max_spd;
            }
            Vector3 target_acc = (target_vel - from_vel) / dt;
            Vector3 true_acc = Vector3.zero;
            if (target_acc.magnitude > max_acc)
            {
                //target acceleration becomes actual acceleration
                true_acc = target_acc.normalized * max_acc;
            }
            else
            {
                true_acc = target_acc;
            }
            //Debug.Log("100x acc we get:" + (100f*true_acc).ToString());
            true_vel = from_vel + dt * true_acc;
            if (true_vel.magnitude > max_spd)
            {
                true_vel = true_vel.normalized * max_spd;
            }
            Vector3 reached_pos = from_v + dt * true_vel;
            Vector3 input = true_acc / max_acc; //matching true acc for weird multplication by max_acc in DroneController

            List<Vector3> Dynamics = new List<Vector3> { reached_pos, true_vel, input };
            return Dynamics;       // Return all data for the reached node
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

        private float GetDistance(Vector3 source_pos, Vector3 target_pos)
        {
            // Returns Euclidian distance.
            return Vector3.Distance(source_pos, target_pos);
        }


        private List<PathTree> GetNeighbors(PathTree source)
        {
            float search_radius = 2f;                                              // TODO: Experiment with this value.
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


        private PathTree RRTStarExpand(Vector3 a_pos, bool relax = false)
        {
            float a_orientation;
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
            float a_velocity = 1.0f;                                // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ TODO: What should a's velocity be? +++++++++++++++++++++++++++++++++++++++++++++++++++++
            //KinematicCarModel a_state = new KinematicCarModel(a_pos, a_orientation, a_velocity);

            // - Find control inputs u to steer the robot from b to a.
            // - Apply control inputs u for time t, so robot reaches c.
            Vector3 c_pos = EasySteer(b, a_pos);

            // - If no collisions occur in getting from b to c:
            if (PathTree.GetNode(c_pos) is null && CheckCollision(b, c_pos) is false)
            {
                //      - Add c as child.
                float b_to_c_cost = GetDistance(b.GetPosition(), c_pos);
                PathTree c = b.AddChild(c_pos, 0.0f, 0.0f, b_to_c_cost);
                ///*
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
                //*/
                return c;
            }
            if (relax)
            {
                return b;
            }
            else
            {
                return null;
            }
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
            float root_orientation = (float)Math.PI * 0.5f;                                       // +++++++++++++++++++++++++++++++ TODO: Find a way to get the orientation of start position. +++++++++++++++++++++++++++++++
            PathTree root = new PathTree(start_pos, 0.0f, root_orientation);      // Start with zero velocity.

            //for (int i = 0; i < iterations; i++)
            while (PathTree.node_dict.Count < tree_size)
            {
                // - Pick a random point a in X.
                Vector3 a_pos;
                //float a_orientation;
                Vector3 rnd_pos;
                while (true)
                {
                    float x_rnd = Random.Range(x_low, x_high);
                    float z_rnd = Random.Range(z_low, z_high);
                    float p_rnd = Random.Range(0.0f, 1.0f);
                    float rnd_trav;

                    if (p_rnd < 0.05f)//bias the search towards the goal
                    {
                        rnd_pos = goal_pos;
                        rnd_trav = 0.0f;

                    }
                    else
                    {
                        int i_rnd = terrain_manager.myInfo.get_i_index(x_rnd);
                        int j_rnd = terrain_manager.myInfo.get_j_index(z_rnd);
                        rnd_pos = new Vector3(x_rnd, 0.0f, z_rnd);
                        rnd_trav = traversability[i_rnd, j_rnd];

                    }

                    if (rnd_trav == 0.0f && PathTree.GetNode(rnd_pos) is null)        // Non-obstacle and not already in tree.
                    {
                        a_pos = rnd_pos;
                        //a_orientation = Random.Range(0, 2 * (float)Math.PI);
                        break;
                    }
                }
                PathTree newest = RRTStarExpand(a_pos);
                if (newest != null && Vector3.Distance(newest.GetPosition(), goal_pos) < goal_thresh)
                {
                    Debug.Log("close enough!");
                    break;
                }
            }

            // Add the goal to the RRT* path.
            //float goal_orientation = -1.0f;                                              // We don't care about the orientation at the goal position.
            PathTree goal = PathTree.GetNode(goal_pos);
            if (goal is null)
            {
                goal = RRTStarExpand(goal_pos, true);
                //throw new Exception("ERROR: Goal was not added to the tree. Try searching for more nodes in RRT*.");
                Debug.Log("old errrorrr");
            }
            // ----------- /RRT* -----------

            suboptimal_path = GetPath(goal);

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

        }


        private void FixedUpdate()
        {
            // Execute your path here.
            float dt = Time.fixedDeltaTime;
            float pi = (float)Math.PI;
            float max_steer = (m_Car.m_MaximumSteerAngle * pi) / 180f;
            float current_vel = m_Car.CurrentSpeed;
            float current_phi = m_Car.CurrentSteerAngle;
            float limit = limitation * (1 - current_vel / m_Car.MaxSpeed);


            
            


            Vector3 true_pos = new Vector3(transform.position.x, 0f, transform.position.z);
            //Debug.Log("transform: " + (transform.position*100f).ToString());
            //Debug.Log("tr pos: " + (true_pos*100f).ToString());

            if (update_count == 0)
            {
               
                MyOldPos = true_pos;
                //Debug.Log("OUR ROOT:" + (suboptimal_path.ElementAt(update_count).GetPosition()).ToString());
            }
            if (update_count < suboptimal_path.Count - 1)
            {
                current_car = suboptimal_path.ElementAt(update_count);
                next_car = suboptimal_path.ElementAt(update_count + 1);
                if (Vector3.Distance(next_car.GetPosition(), true_pos) < optimal_k)
                {
                    update_count++;
                }
                
            }
            else
            {
                next_car = suboptimal_path.ElementAt(suboptimal_path.Count - 1);
                if(Vector3.Distance(true_pos, next_car.GetPosition()) < goal_thresh*5)
                {
                    pwm_switch = true;
                }
            }
            Debug.Log("global orientation" + (global_ori * 180f / pi).ToString());
            
            MyVelocity = (true_pos - MyOldPos) / dt;
            MyOldPos = true_pos;
            //Debug.Log("current vel x100" + (100f * MyVelocity).ToString());
            //Debug.Log("true pos before moves x100" + (100f*true_pos).ToString());
            Vector3 current_pos = current_car.GetPosition();
            Vector3 target_pos = next_car.GetPosition();
            //Debug.Log("target pos x100" + (100f * target_pos).ToString());
            Vector3 input = SteerLive(true_pos, target_pos, MyVelocity)[2]; 
            float turn;
            
            //Debug.Log("mock difference: " + ((float)Math.Atan2(10f - 5f, 0f - 5f) * 180f / pi).ToString());
            float ang_dif = -((float)Math.Atan2(target_pos.z - true_pos.z, target_pos.x - true_pos.x) - global_ori);
            Debug.Log("ANG  DIF  " + ((ang_dif * 180f) / pi).ToString());
            ang_dif = KinematicCarModel.NormalizeAngle(ang_dif);
            Debug.Log("ANG  DIF PROCESSED  " + ((ang_dif * 180f) / pi).ToString());
            if (ang_dif < -max_steer)
            {
                turn = -m_Car.m_MaximumSteerAngle;
            }
            else
            {
                if (ang_dif > max_steer)
                {
                    turn = m_Car.m_MaximumSteerAngle;
                }
                else
                {
                    turn = (ang_dif*180f)/pi;
                }
            }
            
            //This is how you control the car:
            //Debug.Log("acc we send x100" + (100f*input).ToString());
            //Debug.Log("turn we get x100  " + (100f * turn).ToString());
            if(Math.Abs(global_ori - prev_ori) > 5f * pi / 180f)
            {
                pwm = pwm_turn;
            }
            else
            {
                pwm = pwm_steady;
            }

            //Debug.Log("turn we send x100" + (100f * turn / max_steer).ToString());
            if (pwm_count % pwm != 0 || pwm_switch) //||current_vel > 0.2f * m_Car.MaxSpeed )
            {

                m_Car.Move(turn / m_Car.m_MaximumSteerAngle, 0f, 0f, System.Convert.ToSingle(pwm_switch));
            }
            else
            {
                m_Car.Move(turn/ m_Car.m_MaximumSteerAngle, 1f, 0f, 0f);//input.magnitude * limit
                
            }

            Debug.Log("vel " + (current_vel).ToString());
            Debug.Log("MyVel " + (MyVelocity.magnitude).ToString());
            Debug.Log("steer from model  " + (current_phi).ToString());
            float dtheta = -(MyVelocity.magnitude / length) * (float)Math.Tan((current_phi * pi / 180f)) * dt;//-(current_phi * pi / 180f) * dt;//
            Debug.Log("dtheta  " + (dtheta * 180f / pi).ToString());
            prev_ori = global_ori;
            global_ori += dtheta;
            global_ori = KinematicCarModel.NormalizeAngle(global_ori);
            //Debug.Log("true pos after moves 100x" + (100f*transform.position).ToString());
            pwm_count++;
            

        }


        private void DrawRRT()
        {
            foreach (KeyValuePair<Vector3, PathTree> kvp in PathTree.node_dict)
            {
                PathTree node = kvp.Value;
                foreach (PathTree child in node.GetChildren())
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
        private float cost;                                     // Total cost to reach this node.
        private LinkedList<PathTree> children;               // List of this nodes' children.
        private PathTree parent;                             // This nodes' parent.

        public PathTree(Vector3 position, float velocity, float orientation)
        {
            // Constructor for root node.
            this.state = new KinematicCarModel(position, orientation, velocity);
            this.cost = 0f;
            this.children = new LinkedList<PathTree>();
            this.parent = null;
            node_dict.Add(this.state.GetPosition(), this);
        }

        public PathTree(Vector3 position, float velocity, float orientation, float cost)
        {
            // Constructor for non-root nodes.
            this.state = new KinematicCarModel(position, orientation, velocity);
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
            return this.state.GetVelocity();
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
        private float velocity;

        //private static float length = 5.0f;                         // Vehicle length.  (from Group 3)
        //private static float width = 2.5f;                          // Vehicle width.   (from Group 3) --> these are readily seen on the grid!
        private static float length = 4.5f;
        private static float width = 2.5f;
        private static float v_max = 30f;       // Maximum velocity. Keep low to consider approximately constant.
        private static float phi_max = (25f * (float)Math.PI) / 180f;      // Maximum steering angle.

        public KinematicCarModel(Vector3 position, float orientation, float velocity)
        {
            this.position = position;
            this.theta = orientation;
            this.velocity = velocity;
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

        public float GetVelocity()
        {
            return this.velocity;
        }

        public static (float, float) GetInputFromState(float x_dot, float z_dot, float theta_dot)
        {
            // Derive theta 0:
            // v = x / cos(0) = y / sin(0)
            // y = x * sin(0) / cos(0) = x * tan(0)
            // tan(0) = y / x
            // 0 = arctan(y / x)
            float theta = (float)Math.Atan2(z_dot, x_dot);
            float v = x_dot / (float)Math.Cos(theta);
            float phi = (float)Math.Atan2((theta_dot * length), v);
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
}

