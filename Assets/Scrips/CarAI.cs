using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Linq;


namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI : MonoBehaviour
    {
        private CarController m_Car; // the car controller we want to use

        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;
        public double stuck = 0;
        public double turns = 0;

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
            List<Vector2> path_coor = new List<Vector2>();
            /*
            my_path.Add(start_pos);
            Debug.Log(start_pos.ToString());
            for (int i = 0; i < 3; i++)
            {
                Vector3 waypoint = start_pos + new Vector3(UnityEngine.Random.Range(-50.0f, 50.0f), 0, UnityEngine.Random.Range(-30.0f, 30.0f));
                my_path.Add(waypoint);
            }
            my_path.Add(goal_pos);
            */

            my_path.Add(start_pos);
            int start_i = terrain_manager.myInfo.get_i_index(start_pos.x);
            int start_j = terrain_manager.myInfo.get_j_index(start_pos.z);
            int goal_i = terrain_manager.myInfo.get_i_index(goal_pos.x);
            int goal_j = terrain_manager.myInfo.get_j_index(goal_pos.z);
            Debug.Log("start: " + start_pos.ToString() + " = " + "(" + start_i.ToString() + ", " + start_j.ToString() + ")");
            Debug.Log("goal: " + goal_pos.ToString() + " = " + "(" + goal_i.ToString() + ", " + goal_j.ToString() + ")");

            var X = terrain_manager.myInfo.x_N;
            var Z = terrain_manager.myInfo.z_N;
            var trav = terrain_manager.myInfo.traversability;
            var cost = new float[X, Z];
            var selected = new float[X, Z];
            
            
            var xx = 0f;
            var zz = 0f;
            for (int ii = 0; ii < X; ii++)
            {
                for (int jj = 0; jj < Z; jj++)
                {
                    xx = terrain_manager.myInfo.get_x_pos(ii);
                    zz = terrain_manager.myInfo.get_z_pos(jj);
                    cost[ii, jj] = (float)Math.Pow((double)(xx - goal_pos.x), 2) + (float)Math.Pow((double)(zz - goal_pos.z), 2);
                    selected[ii, jj] = 0f;
                }
            }

            for (int ii = 0; ii < X; ii++)
            {
                for (int ji = 0; ji < Z; ji++)
                {
                    Debug.Log("point: " + ii.ToString() + ji.ToString());
                    Debug.Log("trav: " + trav[ii, ji].ToString());
                }
            }

            Debug.Log("X AND Z: " + X.ToString() + " " + Z.ToString());

            selected[start_i, start_j] = 1f;
            //Planning: only for discretized methods (or mixed ones)
            //e.g. A-star/greedy on pixels
            //SEARCH (greedy):
            bool looking = true;
            var last = new Vector3();
            last = my_path.Last();
            Debug.Log(last.ToString());
            IEnumerable<float> enum_cost = cost.Cast<float>();
            do
            {
                int chosen_i = 0;
                int chosen_j = 0;
                float chosen_cost = enum_cost.Max() + 1f;
                for (int a = -1; a <= 1; a++)
                {
                    for (int b = -1; b <= 1; b++)
                    {

                        int candidate_i = terrain_manager.myInfo.get_i_index(last.x) + a;
                        int candidate_j = terrain_manager.myInfo.get_j_index(last.z) + b;
                        Debug.Log("who: " + terrain_manager.myInfo.get_i_index(last.x).ToString() + terrain_manager.myInfo.get_j_index(last.z).ToString());
                        Debug.Log("when: " + a.ToString() + b.ToString());
                        Debug.Log("gives: " + candidate_i.ToString() + candidate_j.ToString());
                        if ((candidate_i < 0 || candidate_i >= X) || (candidate_j < 0 || candidate_j >= Z ) || (trav[candidate_i, candidate_j] > 0) || (selected[candidate_i, candidate_j] >= 1f) || (cost[candidate_i, candidate_j] > chosen_cost))
                        {
                            Debug.Log("skipper: " + candidate_i.ToString() + candidate_j.ToString());
                            //Debug.Log("trav: " + trav[candidate_i, candidate_j].ToString() + " " + (trav[candidate_i, candidate_j] < 1).ToString());
                           

                            continue;
                         }else
                        {
                            chosen_i = candidate_i;
                            chosen_j = candidate_j;
                            chosen_cost = cost[candidate_i, candidate_j];
                            Debug.Log("chosen: " + chosen_i.ToString() + chosen_j.ToString());
                            Debug.Log("chosen trav: " + trav[candidate_i, candidate_j].ToString());
                        }

                    }
                }
                Debug.Log("addin'" + chosen_i.ToString() + chosen_j.ToString());
                selected[chosen_i, chosen_j] = 1f;
                float chosen_xx = terrain_manager.myInfo.get_x_pos(chosen_i);
                float chosen_zz = terrain_manager.myInfo.get_z_pos(chosen_j);
                Vector3 waypoint = new Vector3(chosen_xx, 0f, chosen_zz);
                my_path.Add(waypoint);
                last = waypoint;
                if(chosen_i == goal_i && chosen_j == goal_j)
                {
                    looking = false;
                }
            } while (looking);

            // Plot your path to see if it makes sense
            // Note that path can only be seen in "Scene" window, not "Game" window

            Vector3 old_wp = start_pos;
            foreach (var wp in my_path)
            {
                Debug.DrawLine(old_wp, wp, Color.yellow, 100f);
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


            // this is how you control the car as Move(float steering, float accel, float footbrake, float handbrake)

            float inang = UnityEngine.Random.Range(-0.1f, 0.1f);
            float in_spd = 1f;
            float rev = 1f;
            float spd = m_Car.CurrentSpeed;
            float max_spd = m_Car.MaxSpeed;
            var th = max_spd / 25;
            if (spd <= th)
            {
                stuck = stuck + 1; 
            } else
            {
                turns = 0;
                stuck = 0;
            }
            if(stuck >= 100) { 
                inang = -1f;
                
                in_spd = -0.5f;
                
                if (stuck >= 500 && turns < 500)
                {
                    inang = 1f;
                    turns += 1;
                    in_spd = -0.5f;
                }
          
            } else
            {
                inang = UnityEngine.Random.Range(-0.1f, 0.1f);
            }
            m_Car.Move(inang, in_spd, in_spd, 0f);
            float ang = m_Car.CurrentSteerAngle;
            spd = m_Car.CurrentSpeed;
            var msg = "current angle" + ang.ToString();
            var msg2 = "current speed" + spd.ToString();
            Debug.Log(msg + msg2);
            //var msg3 = "max speed: " + max_spd.ToString() + " thresh: " + th.ToString();
            //Debug.Log(msg3);
            Debug.Log("stuck: " + stuck.ToString() + " turns: " + turns.ToString());
            Debug.Log("(i, j) = (" + i.ToString() + ", " + j.ToString() + ")");
            var xco = transform.position.x;
            var zco = transform.position.z;
            Debug.Log("(x, z) = (" + xco.ToString() + ", " + zco.ToString() + ")");
            

        }
    }
}
