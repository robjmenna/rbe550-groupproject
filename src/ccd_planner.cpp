 #include <pluginlib/class_list_macros.h>
 #include "ccd_planner.h"

 //register this planner as a BaseGlobalPlanner plugin
 PLUGINLIB_EXPORT_CLASS(ccd_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

 using namespace std;

 //Default Constructor
 namespace ccd_planner {

    GlobalPlanner::GlobalPlanner (){

    }

    GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        ROS_INFO("Called constructor");
        initialize(name, costmap_ros);
    }

    void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        ROS_INFO("Called init....");
        
        if (!initialized_)
        {
            costmap_ = costmap_ros->getCostmap();
            costmap_ros_ = costmap_ros;
            ros::param::get("~alpha", alpha);
            ros::param::get("~beta", alpha);
            ros::param::get("~radius", radius);
            ros::param::get("~resolution", resolution);
            ros::param::get("~lethal_threshold", lethal_threshold_);
        }
        else
        {
            ROS_WARN("This planner was already initalized... doing nothing.");
        }
    }

    void GlobalPlanner::computeWavefront(const costmap_2d::MapLocation& goal, std::vector<std::vector<unsigned int>>& wavefront_map)
    {
        std::queue<costmap_2d::MapLocation> queue;
        std::set<unsigned int> visited;

        wavefront_map.clear();
        wavefront_map.resize(costmap_->getSizeInCellsX());
        for (int i=0; i < wavefront_map.size(); i++)
        {
            wavefront_map[i].resize(costmap_->getSizeInCellsY());
        }

        // std::map<int, int> int_cost;
        // wavefront_cost_.clear();
        // int max_cost = 0;
        ROS_INFO("Running the wavefront propagation algorithm.");
        // auto current_index = costmap_->getIndex(start_x, start_y);
        queue.push(goal);
        wavefront_map[goal.x][goal.y] = 0;
        visited.insert(costmap_->getIndex(goal.x, goal.y));
        max_wave_cost = 0;

        while (queue.size() > 0)
        {
            costmap_2d::MapLocation neighbors [8];
            auto current_location = queue.front();
            auto current_x = current_location.x;
            auto current_y = current_location.y;
            // current_index = queue.front();
            queue.pop();
            // costmap_->indexToCells(current_index, current_x, current_y);
            neighbors[0].x = current_x - 1;
            neighbors[0].y = current_y - 1;
            neighbors[1].x = current_x - 1;
            neighbors[1].y = current_y;
            neighbors[2].x = current_x - 1;
            neighbors[2].y = current_y + 1;
            neighbors[3].x = current_x + 1;
            neighbors[3].y = current_y + 1;
            neighbors[4].x = current_x + 1;
            neighbors[4].y = current_y;
            neighbors[5].x = current_x + 1;
            neighbors[5].y = current_y - 1;
            neighbors[6].x = current_x;
            neighbors[6].y = current_y + 1;
            neighbors[7].x = current_x;
            neighbors[7].y = current_y - 1;

            for (int i=0; i < 8; i++)
            {
                if (neighbors[i].x < costmap_->getSizeInCellsX() && neighbors[i].y < costmap_->getSizeInCellsY()
                    && neighbors[i].x >= 0 && neighbors[i].y >= 0)
                {
                    auto cost = costmap_->getCost(neighbors[i].x, neighbors[i].y);
                    auto new_index = costmap_->getIndex(neighbors[i].x, neighbors[i].y);
                    // ROS_INFO("Querying node (%d, %d), which has cost %d", neighbors[i].x, neighbors[i].y, cost);
                    auto ret = visited.insert(new_index);
                    if (ret.second && cost < lethal_threshold_)
                    {
                        queue.push(neighbors[i]);
                        wavefront_map[neighbors[i].x][neighbors[i].y] = wavefront_map[current_x][current_y] + 1;
                        if (wavefront_map[neighbors[i].x][neighbors[i].y] > max_wave_cost)
                        {
                            max_wave_cost = wavefront_map[neighbors[i].x][neighbors[i].y];
                        }
                    }
                }
            }

            // std::map<int, int>::iterator it;
            // for (it = int_cost.begin(); it != int_cost.end(); it++)
            // {
            //     wavefront_cost_[it->first] = ((double)it->second) / ((double)max_cost);
            // }
        }

        // mapped_ = true;
        ROS_INFO("Finished propagating the wave...");
    }

    void GlobalPlanner::markOverlappedAsVisited(const costmap_2d::MapLocation& pos, std::set<int>& visited)
    {
        int cell_radius = 2*(radius / resolution);
        ROS_INFO("Cell radius: %d", cell_radius);
        auto init_pos = -cell_radius;
        auto final_pos = cell_radius + 1;
        for(int i=init_pos; i < final_pos; i++)
        {
            for (int j=init_pos; j < final_pos; j++)
            {
                auto new_x = pos.x + i;
                auto new_y = pos.y + j;
                auto new_index = costmap_->getIndex(new_x, new_y);
                visited.insert(new_index);
            }
        }
    }

    void GlobalPlanner::search_dprime(const costmap_2d::MapLocation& start_location, 
        std::set<int>& visited_sofar, std::vector<geometry_msgs::PoseStamped>& plan,
        nav_msgs::OccupancyGridConstPtr clean_grid)
    {
        std::map<int, int> vertex_table;
        std::set<int> closed;
        std::queue<costmap_2d::MapLocation> queue;
        auto start_index = costmap_->getIndex(start_location.x, start_location.y);
        closed.insert(start_index);
        queue.push(start_location);
        while (queue.size() > 0)
        {
            auto current_location = queue.front();
            auto current_index = costmap_->getIndex(current_location.x, current_location.y);
            queue.pop();
            std::vector<costmap_2d::MapLocation> neighbors;
            getNeighborsFine(current_location.x, current_location.y, neighbors);
            for (int i=0; i < neighbors.size(); i++)
            {
                auto neighbor_index = costmap_->getIndex(neighbors[i].x, neighbors[i].y);
                // auto it = wavefront_cost_.find(current_index);
                auto ret = closed.insert(neighbor_index);
                if (!ret.second)
                {
                    continue;
                }

                // auto closed_it = closed.find(neighbor_index);
                auto cost = costmap_->getCost(neighbors[i].x, neighbors[i].y);
                //todo: move into function
                auto clean_state = clean_grid->data[neighbor_index];
                vertex_table[neighbor_index] = current_index;

                if (clean_state < clean_threshold)
                {
                    visited_sofar.insert(neighbor_index);
                }

                if (cost < lethal_threshold_ && clean_state > clean_threshold)
                {
                    // closed.insert(neighbor_index);
                    // auto neighbor_index = costmap_->getIndex(neighbors[i].x, neighbors[i].y);
                    // auto neighbor_it = visited_sofar.find(neighbor_index);
                    // if (neighbor_it == visited_sofar.end())
                    // {
                    auto next = neighbor_index;
                    std::vector<costmap_2d::MapLocation> new_path;
                    while(next != start_index)
                    {
                        costmap_2d::MapLocation l;
                        // visited_sofar.insert(next);
                        costmap_->indexToCells(next, l.x, l.y);
                        markOverlappedAsVisited(l, visited_sofar);
                        new_path.push_back(l);
                        if (plan.size() + new_path.size() >= max_path_length)
                        {
                            break;
                        }
                        next = vertex_table[next];
                    }

                    for(int i=new_path.size()-1; i >= 0; i--)
                    {
                        appendPose(new_path[i], plan);
                    }
                    return;
                    // }
                    // else
                    // {
                    // }
                }
                queue.push(neighbors[i]);
            }
        }
    }

    void GlobalPlanner::getNeighbors(const unsigned int x, const unsigned int y, 
        const nav_msgs::OccupancyGridConstPtr clean_grid, std::set<int>& visited_sofar, 
        std::vector<costmap_2d::MapLocation>& neighbors)
    {
        // costmap_2d::MapLocation neighbors_ [4];
        int max_radius = 8*(radius / resolution);

        for (int i=0; i < 2; i++)
        {
            // int x_inc = 0;
            for (int j=0; j<max_radius; j++)
            {
                // x_inc++;
                auto x_ = 0;
                if (i == 0)
                {
                    x_ = x + j;
                }
                else
                {
                    x_ = x - j;
                }

                if (x_ < 0 || x_ >= costmap_->getSizeInCellsX())
                {
                    break;
                }

                auto cost = costmap_->getCost(x_, y);
                if (cost >= lethal_threshold_)
                {
                    break;
                }

                auto index = costmap_->getIndex(x_, y);
                auto clean_state = clean_grid->data[index];
                if (clean_state < clean_threshold || visited_sofar.find(index) != visited_sofar.end())
                {
                    visited_sofar.insert(index);
                }
                else
                {
                    costmap_2d::MapLocation temp;
                    temp.x = x_;
                    temp.y = y;
                    neighbors.push_back(temp);
                }

            }
        }

        for (int i=0; i < 2; i++)
        {
            // int y_inc = 0;
            for (int j=0; j<max_radius; j++)
            {
                // y_inc++;
                auto y_ = 0;
                if (i == 0)
                {
                    y_ = y + j;
                }
                else
                {
                    y_ = y - j;
                }
                if (y_ < 0 || y_ >= costmap_->getSizeInCellsY())
                {
                    break;
                }

                auto cost = costmap_->getCost(x, y_);
                if (cost >= lethal_threshold_)
                {
                    break;
                }

                auto index = costmap_->getIndex(x, y_);
                auto clean_state = clean_grid->data[index];
                if (clean_state < clean_threshold || visited_sofar.find(index) != visited_sofar.end())
                {
                    visited_sofar.insert(index);
                }
                else
                {
                    costmap_2d::MapLocation temp;
                    temp.x = x;
                    temp.y = y_;
                    neighbors.push_back(temp);
                }
            }
        }

        // auto cell_radius = ;
        // neighbors_[0].x = x + (cell_radius + 1);
        // neighbors_[0].y = y;
        // neighbors_[1].x = x - (cell_radius + 1);
        // neighbors_[1].y = y;
        // neighbors_[2].x = x;
        // neighbors_[2].y = y + (cell_radius + 1);
        // neighbors_[3].x = x;
        // neighbors_[3].y = y - (cell_radius + 1);

        // for (int i=0; i < 4; i++)
        // {
        //     auto x_good = neighbors_[i].x >= 0 && neighbors_[i].x < costmap_->getSizeInCellsX();
        //     auto y_good = neighbors_[i].y >= 0 && neighbors_[i].y < costmap_->getSizeInCellsY();

        //     if (x_good && y_good)
        //     {
        //         neighbors.push_back(neighbors_[i]);
        //     }
        // }
    }

    void GlobalPlanner::getNeighborsFine(const unsigned int x, const unsigned int y, std::vector<costmap_2d::MapLocation>& neighbors)
    {
        costmap_2d::MapLocation neighbors_ [4];

        neighbors_[0].x = x + 1;
        neighbors_[0].y = y;
        neighbors_[1].x = x - 1;
        neighbors_[1].y = y;
        neighbors_[2].x = x;
        neighbors_[2].y = y + 1;
        neighbors_[3].x = x;
        neighbors_[3].y = y - 1;

        for (int i=0; i < 4; i++)
        {
            auto x_good = neighbors_[i].x >= 0 && neighbors_[i].x < costmap_->getSizeInCellsX();
            auto y_good = neighbors_[i].y >= 0 && neighbors_[i].y < costmap_->getSizeInCellsY();

            if (x_good && y_good)
            {
                neighbors.push_back(neighbors_[i]);
            }
        }
    }

    void GlobalPlanner::appendPose(const costmap_2d::MapLocation& location, std::vector<geometry_msgs::PoseStamped>& plan)
    {
        double x;
        double y;
        // geometry_msgs::PoseStamped new_pose;
        // ROS_INFO("Setting next point at (%d, %d)", next_x, next_y);
        // auto temp_index = costmap_->getIndex(next_x, next_y);
        // visited_.insert(temp_index);
        costmap_->mapToWorld(location.x, location.y, x, y);
        // ROS_INFO("Setting next point at (%f, %f)", x, y);
        // ROS_INFO("%d", (int)plan.size());
        geometry_msgs::PoseStamped current_pose;
        current_pose.header.frame_id = "map";
        current_pose.pose.position.x = x;
        current_pose.pose.position.y = y;

        if (plan.size() == 0)
        {
            auto new_orientation = tf::createQuaternionFromYaw(0);
            current_pose.pose.orientation.x = new_orientation.x();
            current_pose.pose.orientation.y = new_orientation.y();
            current_pose.pose.orientation.z = new_orientation.z();
            current_pose.pose.orientation.w = new_orientation.w();
        }
        else
        {
            auto dx = x - plan.back().pose.position.x;
            auto dy = y - plan.back().pose.position.y;
            auto theta = atan2(dy, dx);
            auto new_orientation = tf::createQuaternionFromYaw(theta);
            current_pose.pose.orientation.x = new_orientation.x();
            current_pose.pose.orientation.y = new_orientation.y();
            current_pose.pose.orientation.z = new_orientation.z();
            current_pose.pose.orientation.w = new_orientation.w();
        }

        plan.push_back(current_pose);
    }

    void GlobalPlanner::planPath(const costmap_2d::MapLocation& start, const costmap_2d::MapLocation& goal, 
        std::vector<geometry_msgs::PoseStamped>& plan,
        std::vector<std::vector<unsigned int>> wavefront_map){
    
        // std::vector<costmap_2d::MapLocation> map_coords;
        std::set<int> visited;

        // computeWavefront(goal, wavefront_map);
        markOverlappedAsVisited(start, visited);
        auto clean_grid = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/coverage_grid", ros::Duration(5));
        // markOverlappedAsVisited(map_location.x, map_location.y, local_visited);
        // plan.push_back(start);
        // plan.push_back(robot_pose);
        // map_coords.push_back(map_location);
        // auto foo = costmap_->getIndex(map_location.x, map_location.y);
        // visited_.insert(foo);
        plan.clear();
        auto current_location = start;
        while (true)
        {
            // unsigned int next_index;
            // double cost = 0;
            double max_cost = -99999;
            costmap_2d::MapLocation next_location;

            // unsigned int next_x;
            // unsigned int next_y;
            std::vector<costmap_2d::MapLocation> neighbors;

            auto current_x = current_location.x;
            auto current_y = current_location.y;
            getNeighbors(current_x, current_y, clean_grid, visited, neighbors);

            // neighbors[0][0] = current_x + cell_radius + 2;
            // neighbors[0][1] = current_y;
            // neighbors[1][0] = current_x - cell_radius + 2;
            // neighbors[1][1] = current_y;
            // neighbors[2][0] = current_x;
            // neighbors[2][1] = current_y + cell_radius + 2;
            // neighbors[3][0] = current_x;
            // neighbors[3][1] = current_y - cell_radius + 2;

            for (int i=0; i < neighbors.size(); i++)
            {

                // costmap_->worldToMapEnforceBounds(neighbors[i][0], neighbors[i][1], neighbor_x, neighbor_y);
                // ROS_INFO("Testing neighbor at (%d, %d)", neighbors[i][0], neighbors[i][1]);
                auto temp_index = costmap_->getIndex(neighbors[i].x, neighbors[i].y);
                // auto it = wavefront_cost_.find(temp_index);
                auto cost = costmap_->getCost(neighbors[i].x, neighbors[i].y);

                // auto foo = costmap_ros_->getLayeredCostmap()->getPlugins();
                // for (int j=0; j < foo->size(); j++)
                // {
                //     auto goo = (*foo)[j]->getName();
                //     ROS_INFO("Found layer %s", goo);
                // }
                auto clean_state = clean_grid->data[temp_index];

                if (clean_state < clean_threshold)
                {
                    visited.insert(temp_index);
                }

                auto v_it = visited.find(temp_index);
                // auto lv_it = local_visited.find(temp_index);
                // auto foo = costmap_->getCost(neighbors[i][0], neighbors[i][1]);
                // ROS_INFO("%d", it != wavefront_cost_.end());
                if (cost < lethal_threshold_ && v_it == visited.end())
                {
                    // auto weighted_cost = wavefront_cost_[temp_index] - alpha_*((double)costmap_->getCost(neighbors[i][0], neighbors[i][1]) / 255.0);
                    auto wave_cost = beta*wavefront_map[neighbors[i].x][neighbors[i].y];
                    double obs_cost = alpha*lethal_threshold_;
                    int weighted_cost = wave_cost - obs_cost;
                    ROS_INFO("Found cost %d; wave cost %d; obs cost: %d", weighted_cost, wave_cost, obs_cost);
                    // auto weighted_cost = wavefront_cost_[temp_index] + costmap_->getCost(neighbors[i][0], neighbors[i][1]);
                    if (weighted_cost > max_cost)
                    {
                        max_cost = weighted_cost;
                        next_location = neighbors[i];
                    }
                }
            }

            if (max_cost == 0)
            {
                ROS_INFO("Executing the dprime search...");
                // return true;
                auto current_size = plan.size();
                search_dprime(next_location, visited, plan, clean_grid);

                if (current_size == plan.size())
                {
                    // plan.push_back(goal);
                    ROS_INFO("Everything is covered! %d", (int)current_size);
                    return;
                }
                else
                {
                    auto last_pose = plan.back();
                    costmap_->worldToMap(last_pose.pose.position.x, last_pose.pose.position.y, current_location.x, current_location.y);
                    // map_coords.push_back(next_location);
                }
            }
            else
            {
                appendPose(next_location, plan);
                // auto new_index = costmap_->getIndex(next_location.x, next_location.y);
                // visited_.insert(new_index);
                markOverlappedAsVisited(next_location, visited);
                current_location = next_location;
            }
            
            // if (plan.size() >= max_path_length)
            // {
            //     ROS_INFO("Reached max plan length: %d", max_path_length);
            //     return;
            // }
        }
    }

    bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan ){        
        costmap_2d::MapLocation start_loc;
        costmap_2d::MapLocation goal_loc;
        
        costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_loc.x, start_loc.y);
        costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_loc.x, goal_loc.y);
        if (last_goal.x != goal_loc.x || last_goal.y != goal_loc.y)
        {
            computeWavefront(goal_loc, wavefront_map_);
            last_goal = goal_loc;
        }

        planPath(start_loc, goal_loc, plan, wavefront_map_);
        return true;
    }

    // bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan ){        
    //     // std::set<int> local_visited;

    //     if (goal.pose.position.x != last_goal.position.x || goal.pose.position.y != last_goal.position.y)
    //     {
    //         geometry_msgs::PoseStamped robot_pose;
    //         costmap_2d::MapLocation map_location;
    //         unsigned int goal_x;
    //         unsigned int goal_y;
    //         last_goal = goal.pose;
    //         // visited_.clear();
    //         costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y);
    //         wavefront_cost_.clear();
    //         computeWavefront(goal_x, goal_y);
    //         costmap_ros_->getRobotPose(robot_pose);
    //         costmap_->worldToMap(robot_pose.pose.position.x, robot_pose.pose.position.y, map_location.x, map_location.y);
    //         plan.push_back(start);
    //         planPath(map_location, plan);
    //         last_plan = plan;
    //     }
    //     else{
    //         costmap_2d::MapLocation map_location;
    //         costmap_2d::MapLocation start_location;
    //         costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_location.x, start_location.y);
    //         costmap_->worldToMap(plan[0].pose.position.x, plan[0].pose.position.y, map_location.x, map_location.y);
    //         int index = 0;
    //         while(start_location.x != map_location.x && start_location.y != map_location.y)
    //         {
    //             index++;
    //             costmap_->worldToMap(plan[index].pose.position.x, plan[index].pose.position.y, map_location.x, map_location.y);
    //         }

    //         while(index < last_plan.size())
    //         {
    //             plan.push_back(last_plan[index]);
    //             index++;
    //         }

    //         last_plan = plan;
    //     }
    //     return true;

    //     // ROS_INFO("Found the robot at (%d, %d)", map_location.x, map_location.y);
    //     // if (!mapped_){
    //     //     // costmap_2d::MapLocation goal_cell;
    //     //     ROS_INFO("Propagating wave from (%d, %d)", goal_x, goal_y);
    //     //     ROS_INFO("Map size: (%d,%d)", costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
    //     // }


        
    // }
 };