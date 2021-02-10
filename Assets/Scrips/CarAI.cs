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
        bool DEBUG_RRT_LIVE = false;
        bool DEBUG_COLLISION = false;
        int update_count = 0;
        int overshooter = 0;
        LinkedList<PathTree<Vector3>> optimal_path;
        Vector3 current_updatee;
        float acc_in;
        float str_in;

        private CarController m_Car; // the car controller we want to use
        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;

        private Vector3 Steer(PathTree<Vector3> source, Vector3 target_pos)
        {
            // The Reeds-Shepp/Dubins curves are calculated in the steer function.
            float d = Vector3.Distance(source.GetPosition(), target_pos);
            Vector3 reached_pos = target_pos;
            if(d > 5f)
            {
                reached_pos = Vector3.MoveTowards(source.GetPosition(), target_pos, 5f);
            }
            return reached_pos;       // Temporarily just return a.
        }

        private bool CheckCollision(PathTree<Vector3> source, Vector3 target_pos)
        {
            Vector3 current_pos = source.GetPosition();
            Vector3 collision_pos;
            float step = 0.05f * GetDistance(current_pos, target_pos);
            int current_i;
            int current_j;
            float length = 2.5f;
            List<Vector3> tilings = new List<Vector3>{Vector3.zero, new Vector3(length, 0, 0), new Vector3(-length, 0, 0), new Vector3(0, 0, length), new Vector3(0, 0, -length) };
            
            

            //Debug.Log("---------");
            //Debug.Log("Source position: " + current_pos.ToString());
            //Debug.Log("Target position: " + target_pos.ToString());

            while (current_pos != target_pos)
            {
                current_pos = Vector3.MoveTowards(current_pos, target_pos, step);
                foreach(Vector3 tile in tilings)
                {
                    collision_pos = current_pos + tile;
                    current_i = terrain_manager.myInfo.get_i_index(collision_pos[0]);
                    current_j = terrain_manager.myInfo.get_j_index(collision_pos[2]);
                    if (terrain_manager.myInfo.traversability[current_i, current_j] == 1)       // Collision.
                    {
                        Debug.Log("Collision found: " + collision_pos.ToString() + " at i = " + current_i.ToString() + ", j = " + current_j.ToString());
                        return true;
                    }
                   
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

    private PathTree<Vector3> RRTStarExpand(Vector3 a_pos, bool relax = false)
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
        /*
        List<Vector3> c_dynamics = Steer(b, a_pos);
        Vector3 c_pos = c_dynamics[0];
        Vector3 c_vel = c_dynamics[1];
        Vector3 c_acc = c_dynamics[2];
        */
        if (PathTree<Vector3>.GetNode(c_pos) != null)
        {
            Debug.Log("c is already there, trying again (c is as follows): " + c_pos.ToString());
            return null;
        }
        //Debug.Log("we made it to the new: " + c_pos.ToString());

        // - If no collisions occur in getting from b to c:
        if (CheckCollision(b, c_pos) is false)
        {
            //      - Add c as child.
            float b_to_c_cost = GetDistance(b.GetPosition(), c_pos);
           // PathTree<Vector3> c = b.AddChild(c_pos, c_vel, c_acc, b_to_c_cost);
           PathTree<Vector3> c = b.AddChild(c_pos, b_to_c_cost);

            /*
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
            //*/

        /*
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

            int tree_size = 10000;
            PathTree<Vector3> root = new PathTree<Vector3>(start_pos);
            while (PathTree<Vector3>.node_dict.Count < tree_size)
            {
                // - Pick a random point a in X.
                Vector3 a_pos;
                Vector3 rnd_pos;
                while (true)
                {
                    float x_rnd = Random.Range(x_low, x_high);
                    float z_rnd = Random.Range(z_low, z_high);
                    float p_rnd = Random.Range(0.0f, 1.0f);
                    float rnd_trav;
                    if (p_rnd < 0.1f)
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

                    if (rnd_trav == 0.0f && PathTree<Vector3>.GetNode(rnd_pos) is null)        // Non-obstacle and not already in tree.
                    {
                        a_pos = rnd_pos;
                        //Debug.Log("our suitable goal is: " + a_pos.ToString());
                        break;
                    }
                }

                PathTree<Vector3> newest = RRTStarExpand(a_pos);
                if (newest != null && Vector3.Distance(newest.GetPosition(), goal_pos) < 1f)
                {
                    Debug.Log("close enough!");
                    break;
                }
            }

            PathTree<Vector3> goal = PathTree<Vector3>.GetNode(goal_pos);
            // Add the goal to the RRT* path.
            if (goal is null)
            {
                goal = RRTStarExpand(goal_pos, true);
                //throw new Exception("ERROR: Goal was not added to the tree. Try searching for more nodes in RRT*.");
                Debug.Log("old errrorrr");
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


            // Plot your path to see if it makes sense
            // Note that path can only be seen in "Scene" window, not "Game" window
            //Vector3 old_wp = start_pos;
            //    foreach (var wp in my_path)
            //    {
            //        Debug.DrawLine(old_wp, wp, Color.red, 100f);
            //        old_wp = wp;
            //    }
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
            //L (decent estimate) = 4
            
            if(update_count == 0)
            {
                acc_in = 0.25f;//Random.Range(0f, 0.35f);
                str_in = 1f;// Random.Range(-0.5f, 0f);
            }
            if(update_count > 50)
            {
                acc_in = 0.25f;//Random.Range(0f, 0.35f);
                str_in = 0f;// Random.Range(-0.5f, 0f);
            }
            m_Car.Move(str_in, acc_in, acc_in, 0f);
            Debug.Log("current steer: " + m_Car.CurrentSteerAngle + "max: " + m_Car.m_MaximumSteerAngle);
            Debug.Log("current speed: " + m_Car.CurrentSpeed + "maxi: " + m_Car.MaxSpeed);
            update_count++;

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
}
