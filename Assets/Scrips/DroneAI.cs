using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(DroneController))]
public class DroneAI : MonoBehaviour
{
    bool DEBUG_RRT_LIVE = false;
    bool DEBUG_COLLISION = false;

    private DroneController m_Drone; // the car controller we want to use
 
    public GameObject terrain_manager_game_object;
    TerrainManager terrain_manager;

    private List<Vector3> Steer(PathTree<Vector3> source, Vector3 target_pos)
    {
        Debug.Log("Steeeer");
        float dt = Time.fixedDeltaTime;
        float max_spd = m_Drone.max_speed;
        float max_acc = m_Drone.max_acceleration;
        Vector3 current_pos = source.GetPosition();
        Vector3 reached_pos = Vector3.zero;
        Vector3 current_vel = source.GetVelocity();
        Debug.Log("target pos: " + target_pos.ToString() + " current: " + current_pos.ToString());
        Vector3 target_vel = (target_pos - current_pos) / dt;
        Debug.Log("target vel: " + target_vel.ToString() + " current: " + current_vel.ToString());
        Vector3 true_vel = Vector3.zero;
        if (target_vel.magnitude > max_spd)
        {
            target_vel = target_vel.normalized * max_spd;
        }
        Debug.Log("filtered target vel: " + target_vel.ToString());
        Vector3 target_acc = (target_vel - current_vel) / dt;
        Debug.Log("target acc: " + target_acc.ToString());
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

        true_vel = current_vel + dt * true_acc;
        if (true_vel.magnitude > max_spd)
        {
            true_vel = true_vel.normalized * max_spd;
        }
        reached_pos = current_pos + dt * true_vel;
        Debug.Log("true acc! " + true_acc.ToString());
        Debug.Log("true vel! " + true_vel.ToString());
        Debug.Log("true reached pos! " + reached_pos.ToString());

        Vector3 input = true_acc / max_acc; //matching true acc for weird multplication by max_acc in DroneController

        List<Vector3> Dynamics = new List<Vector3> { reached_pos, true_vel, input };
        return Dynamics;       // Temporarily just return a.
    }

    private bool CheckCollision(PathTree<Vector3> source, Vector3 target_pos)
    {
        if (DEBUG_COLLISION)
        {
            return false;       // Disables collision detection.
        }

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

    private void Start()
    {
        // get the drone controller
        m_Drone = GetComponent<DroneController>();
        terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();


        Vector3 start_pos = terrain_manager.myInfo.start_pos;
        Vector3 goal_pos = terrain_manager.myInfo.goal_pos;

        List<Vector3> my_path = new List<Vector3>();

        // Plan your path here
        // ...
        my_path.Add(start_pos);
        // ----------- Initializations -----------

        float x_low = terrain_manager.myInfo.x_low;
        float x_high = terrain_manager.myInfo.x_high;
        float z_low = terrain_manager.myInfo.z_low;
        float z_high = terrain_manager.myInfo.z_high;
        var traversability = terrain_manager.myInfo.traversability;

        // ----------- RRT* -----------

        int iterations = 5000;
        PathTree<Vector3> root = new PathTree<Vector3>(start_pos, Vector3.zero, Vector3.zero);//start with zero velocity and no input
        
        

        for (int i = 0; i < iterations; i++)
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
                        Debug.Log("our suitable goal is: " + a_pos.ToString());
                        break;
                }
            }

            // - Find b, the node of the tree closest to a.
            //PathTree<Vector3> b = root;
            float dist;
            float min_dist = float.PositiveInfinity;
            PathTree<Vector3> b = root;
            foreach (KeyValuePair<Vector3, PathTree<Vector3>> kvp in PathTree<Vector3>.node_dict)
            {
                dist = GetDistance(kvp.Key, a_pos);
                if (dist < min_dist)
                {
                    min_dist = dist;
                    b = kvp.Value;
                        Debug.Log("we is starting from: " + b.GetPosition().ToString());
                }
            }

            // - Find control inputs u to steer the robot from b to a.
            // - Apply control inputs u for time t, so robot reaches c.
            List<Vector3> c_dynamics = Steer(b, a_pos);
            Vector3 c_pos = c_dynamics[0];
            Vector3 c_vel = c_dynamics[1];
            Vector3 c_acc = c_dynamics[2];
            if(PathTree<Vector3>.GetNode(c_pos) != null)
            {
                Debug.Log("c is already there, trying again (c is as follows): " + c_pos.ToString());
                break;
            }
                Debug.Log("we made it to the new: " + c_pos.ToString());

                // - If no collisions occur in getting from b to c:
                if (CheckCollision(b, c_pos) is false)
            {
                //      - Add c as child.
                float b_to_c_cost = GetDistance(b.GetPosition(), c_pos);
                PathTree<Vector3> c = b.AddChild(c_pos, c_vel, c_acc, b_to_c_cost);

                //      - Find set of Neighbors N of c.
                //List<PathTree<Vector3>> c_neighbors = GetNeighbors(c);

                //      - Choose Best parent.
                /*
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
                */

                //      - Try to adopt Neighbors (if good).
                /*
                foreach (PathTree<Vector3> c_neighbor in c_neighbors)
                {
                    float c_to_neighbor_cost = GetDistance(c.GetPosition(), c_neighbor.GetPosition());
                    float neighbor_cost_new = c.GetCost() + c_to_neighbor_cost;
                    if (CheckCollision(c, c_neighbor.GetPosition()) is false && neighbor_cost_new < c_neighbor.GetCost())
                    {
                        c.AdoptChild(c_neighbor, neighbor_cost_new);
                    }
                }
                */

            }
        }
        // ----------- Draw RRT* Path -----------

        if (DEBUG_RRT_LIVE)
        {
            StartCoroutine(DrawRRTLive(root));                  // Draw the RRT* path LIVE.
        }
        else
        {
            DrawRRT();                                          // Draw the whole RRT* path immediately.
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

        Debug.DrawLine(transform.position, new Vector3(grid_center_x, 0f, grid_center_z), Color.white, 1f);

        //Debug.Log("di delta: " + Time.fixedDeltaTime.ToString());
        // this is how you control the car
        m_Drone.Move(0.4f * Mathf.Sin(Time.time * 1.9f), 0.1f);

    }


    private void DrawRRT()
    {
        foreach (KeyValuePair<Vector3, PathTree<Vector3>> kvp in PathTree<Vector3>.node_dict)
        {
            PathTree<Vector3> node = kvp.Value;
            foreach (PathTree<Vector3> child in node.GetChildren())
            {
                Debug.DrawLine(node.GetPosition(), child.GetPosition(), Color.red, 100f);
            }
        }
    }


    IEnumerator DrawRRTLive(PathTree<Vector3> root)
    {
        // Draws the path live with a BFS search.
        WaitForSeconds wait = new WaitForSeconds(0.001f);                 // Wait time between lines being drawn.
        LinkedList<PathTree<Vector3>> queue = new LinkedList<PathTree<Vector3>>();
        queue.AddLast(root);

        while (true)
        {
            PathTree<Vector3> node = queue.First.Value;
            queue.RemoveFirst();
            foreach (PathTree<Vector3> child in node.GetChildren())
            {
                queue.AddLast(child);
                Debug.DrawLine(node.GetPosition(), child.GetPosition(), Color.red, 100f);
                yield return wait;
            }
            if (queue.Last is null)
            {
                break;
            }
        }
    }
}


class PathTree<T>
{
    public static Dictionary<T, PathTree<T>> node_dict = new Dictionary<T, PathTree<T>>();
    private T position;
    private T velocity;
    private T input;                                       // Position of this node.
    private float cost;                                     // Total cost to reach this node.
    private LinkedList<PathTree<T>> children;               // List of this nodes' children.
    private PathTree<T> parent;                             // This nodes' parent.

    public PathTree(T position, T velocity, T input)
    {
        // Constructor for root node.
        this.position = position;
        this.velocity = velocity;
        this.input = input;
        this.cost = 0f;
        this.children = new LinkedList<PathTree<T>>();
        this.parent = null;
        node_dict.Add(this.position, this);
    }

    public PathTree(T position, T velocity, T input, float cost)
    {
        // Constructor for non-root nodes.
        this.position = position;
        this.velocity = velocity;
        this.input = input;
        this.cost = cost;
        this.children = new LinkedList<PathTree<T>>();
        node_dict.Add(this.position, this);
    }

    public PathTree<T> AddChild(T position, T velocity, T input, float sub_cost)
    {
        float child_cost = this.cost + sub_cost;
        PathTree<T> new_child = new PathTree<T>(position, velocity, input, child_cost);
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

    public T GetVelocity()
    {
        return this.velocity;
    }

    public float GetCost()
    {
        return this.cost;
    }

    public T GetInput()
    {
        return this.input;
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
