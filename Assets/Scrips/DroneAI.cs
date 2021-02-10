using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

[RequireComponent(typeof(DroneController))]
public class DroneAI : MonoBehaviour
{
    bool DEBUG_RRT_LIVE = false;
    bool DEBUG_COLLISION = false;
    int update_count = 0;
    int overshooter = 0;
    Vector3 total_error = Vector3.zero;
    Vector3 error = Vector3.zero;
    Vector3 prev_error = Vector3.zero;
    List<Vector3> nasta_up = new List<Vector3>();
    LinkedList<List<Vector3>> optimal_path = new LinkedList<List<Vector3>>();
    LinkedList<PathTree<Vector3>> suboptimal_path;
    Vector3 current_updatee;
    Vector3 current_spedee;
    float D;
    float P;
    float I;
    private DroneController m_Drone; // the car controller we want to use

    public GameObject terrain_manager_game_object;
    TerrainManager terrain_manager;

    private List<Vector3> Steer(PathTree<Vector3> source, Vector3 target_pos)
    {
        //Debug.Log("Steeeer");
        /*
        float dt = Time.fixedDeltaTime;
        float max_spd = m_Drone.max_speed*0.8f;
        float max_acc = m_Drone.max_acceleration;
        */
        Vector3 current_pos = source.GetPosition();
        Vector3 reached_pos = Vector3.zero;
        Vector3 current_vel = source.GetVelocity();
        if(Vector3.Distance(current_pos, target_pos) > 2f) 
        {
            target_pos = Vector3.MoveTowards(current_pos, target_pos, 2f);//modest goal
        }
        /*
        //Debug.Log("target pos: " + target_pos.ToString() + " current: " + current_pos.ToString());
        Vector3 target_vel = (target_pos - current_pos) / dt;
        //Debug.Log("target vel: " + target_vel.ToString() + " current: " + current_vel.ToString());
        Vector3 true_vel = Vector3.zero;
        if (target_vel.magnitude > max_spd)
        {
            target_vel = target_vel.normalized * max_spd;
        }
        //Debug.Log("filtered target vel: " + target_vel.ToString());
        Vector3 target_acc = (target_vel - current_vel) / dt;
        //Debug.Log("target acc: " + target_acc.ToString());
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
        //Debug.Log("true acc! " + true_acc.ToString());
        //Debug.Log("true vel! " + true_vel.ToString());
        //Debug.Log("true reached pos! " + reached_pos.ToString());
        Vector3 input = true_acc/max_acc; //matching true acc for weird multplication by max_acc in DroneController
        List<Vector3> Dynamics = new List<Vector3> { reached_pos, true_vel, input };
        return Dynamics;       // Return all data for the reached node
        //*/
        ///*
        List<Vector3> Dummy = new List<Vector3> {target_pos, current_vel, Vector3.zero};
        return Dummy;
        //*/
    }

    private List<Vector3> SteerLive(Vector3 from_v, Vector3 to_v, Vector3 from_vel)
    {
        Debug.Log("Steeeer");
        float dt = Time.fixedDeltaTime;
        float max_spd = m_Drone.max_speed * 0.5f;
        float max_acc = m_Drone.max_acceleration;
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

    private bool CheckCollision(PathTree<Vector3> source, Vector3 target_pos)
    {
        Vector3 current_pos = source.GetPosition();
        Vector3 collision_pos;
        float step = 0.05f * GetDistance(current_pos, target_pos);
        int current_i;
        int current_j;
        float length = 3.0f;
        List<Vector3> tilings = new List<Vector3> { Vector3.zero, new Vector3(length, 0, 0), new Vector3(-length, 0, 0), new Vector3(0, 0, length), new Vector3(0, 0, -length) };//,
                                                    //new Vector3(length, 0, length), new Vector3(-length, 0, -length), new Vector3(-length, 0, length), new Vector3(length, 0, -length)};



        //Debug.Log("---------");
        //Debug.Log("Source position: " + current_pos.ToString());
        //Debug.Log("Target position: " + target_pos.ToString());

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
        float search_radius = 2f;                                              // TODO: Experiment with this value.
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
        List<Vector3> c_dynamics = Steer(b, a_pos);
        Vector3 c_pos = c_dynamics[0];
        Vector3 c_vel = c_dynamics[1];
        Vector3 c_acc = c_dynamics[2];

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
            PathTree<Vector3> c = b.AddChild(c_pos, c_vel, c_acc, b_to_c_cost);

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
        int tree_size = 10000;
        PathTree<Vector3> root = new PathTree<Vector3>(start_pos, Vector3.zero, Vector3.zero);//start with zero velocity and no input

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
                float p_rnd = Random.Range(0.0f, 1.0f);
                float rnd_trav;
                if (p_rnd < 0.05f)
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
        if (goal is null)
        {
            goal = RRTStarExpand(goal_pos, true);
            //throw new Exception("ERROR: Goal was not added to the tree. Try searching for more nodes in RRT*.");
            Debug.Log("old errrorrr");
        }
        /*
        for (int i = 0; i < 100; i++)
        {
            if (goal is null || (Vector3.Distance(goal.GetPosition(), goal_pos) > 1f))
            {
                goal = RRTStarExpand(goal_pos);
            }
            else
            {
                Debug.Log("succesful goal addition, break!");
                break;
            }

        }
        if (goal is null)
        {
            Debug.Log("had to do it myself...");
            goal = RRTStarExpand(goal_pos, true);
        }
        */
        /*
        while (goal is null)
        {
            Debug.Log("trying to get valid goal.");
            goal = RRTStarExpand(goal_pos, true);
        }
        if (goal is null)
        {
            Debug.Log("not worked!!");
        }
        else
        {
            Debug.Log("worked!!");
        }
        */
        




        // ----------- Draw RRT* Path -----------

        if (DEBUG_RRT_LIVE)
        {
            StartCoroutine(DrawRRTLive(root));                  // Draw the RRT* path LIVE.
        }
        else
        {
            DrawRRT();                                          // Draw the whole RRT* path immediately.
            DrawPath(goal);
        }

        //----------add dynamic constraints to path-----
        suboptimal_path = GetPath(goal);
        PathTree<Vector3> current = suboptimal_path.ElementAt(0);
        PathTree<Vector3> nasta = suboptimal_path.ElementAt(1);
        List<Vector3> reached = new List<Vector3>();
        reached = SteerLive(current.GetPosition(), nasta.GetPosition(), current.GetVelocity());
        List<Vector3> opt_root = new List<Vector3> { current.GetPosition(), current.GetVelocity(), Vector3.zero };
        optimal_path.AddLast(opt_root);
        optimal_path.AddLast(reached);
        for (int k = 1; k < suboptimal_path.Count - 1; k++)
        {
            nasta = suboptimal_path.ElementAt(k);
            while(Vector3.Distance(reached[0], nasta.GetPosition()) > 0.5f) //&& (Vector3.Distance(reached[0], goal.GetPosition()) > (Vector3.Distance(nasta.GetPosition(), goal.GetPosition()))))
            {
                reached = SteerLive(reached[0], nasta.GetPosition(), reached[1]);
                optimal_path.AddLast(reached);
            }
        }
        //----------add dynamic constraints to path-----
        //---------draw obtained path------------------
        for (int k = 1; k < optimal_path.Count - 1; k++)
        {
            Debug.DrawLine(optimal_path.ElementAt(k - 1)[0], optimal_path.ElementAt(k)[0], Color.green, 100f);
        }
        
        //---------draw obtained path------------------



    }


    private void FixedUpdate()
    {
        // Execute your path here
        // ...

        // this is how you access information about the terrain
        /*
        int i = terrain_manager.myInfo.get_i_index(transform.position.x);
        int j = terrain_manager.myInfo.get_j_index(transform.position.z);
        float grid_center_x = terrain_manager.myInfo.get_x_pos(i);
        float grid_center_z = terrain_manager.myInfo.get_z_pos(j);
        */

        float dt = Time.fixedDeltaTime;
        Vector3 start_pos = terrain_manager.myInfo.start_pos;
        Vector3 goal_pos = terrain_manager.myInfo.goal_pos;

        //Debug.DrawLine(transform.position, new Vector3(grid_center_x, 0f, grid_center_z), Color.white, 1f);

        //Debug.Log("di delta: " + Time.fixedDeltaTime.ToString());
        // this is how you control the car


        //PathTree<Vector3> rut = optimal_path.ElementAt(0);
        //PathTree<Vector3> goal = optimal_path.ElementAt(optimal_path.Count - 1);
        List<Vector3> goal = optimal_path.ElementAt(optimal_path.Count - 1);

        if (update_count == 0)
        {
            Debug.Log("size of path: " + suboptimal_path.Count.ToString());
            Debug.Log("size of OPT path: " + optimal_path.Count.ToString());
            nasta_up = optimal_path.ElementAt(update_count + 1);
            //current_updatee = transform.position;
            // P = 1.5f;
            //I = 0.25f;
            // D = 0f;
        }
        if (Vector3.Distance(transform.position, nasta_up[0]) < 0.2f || update_count == 0 ||true) //&& (Vector3.Distance(transform.position, goal[0]) > (Vector3.Distance(nasta_up[0], goal[0]))))
        {
            update_count += 1;
        }
        //PathTree<Vector3> current = optimal_path.ElementAt(update_count);
        //List<Vector3> current = optimal_path.ElementAt(update_count);
        if (update_count < (optimal_path.Count - 1))
        {
            //PathTree<Vector3> nasta = optimal_path.ElementAt(update_count + 1);
            
            nasta_up = optimal_path.ElementAt(update_count + 1);
            //Debug.Log("true current" + current_updatee.ToString());
            Debug.Log("true current SURE" + transform.position.ToString());
            //Debug.Log("going to: " + nasta.GetPosition());
            Debug.Log("going to: " + nasta_up[0].ToString());
            Debug.Log("distance: " + Vector3.Distance(transform.position, nasta_up[0]).ToString());
            Debug.Log("update count: " + update_count.ToString());
            //Debug.Log("ctrl insss: " + current_updatee.GetInput().ToString());
            //Debug.Log("vel should be: " + current_updatee.GetVelocity().ToString());
            //Debug.Log("but is " + m_Drone.velocity);
            List<Vector3> real_answers = SteerLive(transform.position, nasta_up[0], m_Drone.velocity);
            //Vector3 ctrl_input = current.GetInput();
            //error = nasta[0] - transform.position;
            //total_error = total_error + error;

            Vector3 ctrl_input = real_answers[2];
            //Vector3 ctrl_input = P*error + I*total_error*dt + D*(error - prev_error)/dt;
            m_Drone.Move(ctrl_input.x, ctrl_input.z);
            //prev_error = error;
            //current_updatee = transform.position;
            
            
        }
        else
        {
            Debug.Log("APPARENTLY DONE");
            //List<Vector3> real_answers = SteerLive(transform.position, goal.GetPosition(), m_Drone.velocity);
            List<Vector3> real_answers = SteerLive(transform.position, goal[0], m_Drone.velocity);
            //Vector3 ctrl_input = current.GetInput();
            Vector3 ctrl_input = real_answers[2];
            m_Drone.Move(ctrl_input.x, ctrl_input.z);
            //current_updatee = transform.position;
            overshooter++;
        }


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
