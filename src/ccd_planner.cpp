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
            alpha_ = 0.5;
        }
        else
        {
            ROS_WARN("This planner was already initalized... doing nothing.");
        }
    }

    void GlobalPlanner::computeWavefront(const unsigned int start_x, const unsigned int start_y)
    {
        std::queue<unsigned int> queue;
        std::set<unsigned int> visited;
        // unsigned int start_x;
        // unsigned int start_y;
        unsigned int current_x = start_x;
        unsigned int current_y = start_y;
        unsigned int neighbors [8][2];
        unsigned char cost;
        unsigned int new_index;
        std::pair<std::set<unsigned int>::iterator, bool> ret;
        unsigned int current_index;

        ROS_INFO("Running the wavefront propagation algorithm.");
        current_index = costmap_->getIndex(current_x, current_y);
        queue.push(current_index);
        wavefront_cost_[current_index] = 0;
        visited.insert(current_index);
        while (queue.size() > 0)
        {
            current_index = queue.front();
            queue.pop();
            costmap_->indexToCells(current_index, current_x, current_y);
            neighbors[0][0] = current_x - 1;
            neighbors[0][1] = current_y - 1;
            neighbors[1][0] = current_x - 1;
            neighbors[1][1] = current_y;
            neighbors[2][0] = current_x - 1;
            neighbors[2][1] = current_y + 1;
            neighbors[3][0] = current_x + 1;
            neighbors[3][1] = current_y + 1;
            neighbors[4][1] = current_x + 1;
            neighbors[4][1] = current_y;
            neighbors[5][1] = current_x + 1;
            neighbors[5][1] = current_y - 1;
            neighbors[6][1] = current_x;
            neighbors[6][1] = current_y + 1;
            neighbors[7][1] = current_x;
            neighbors[7][1] = current_y - 1;

            for (int i=0; i < 8; i++)
            {
                if (neighbors[i][0] < costmap_->getSizeInCellsX() && neighbors[i][1] < costmap_->getSizeInCellsY()
                    && neighbors[i][0] >= 0 && neighbors[i][1] >= 0)
                {
                    cost = costmap_->getCost(neighbors[i][0], neighbors[i][1]);
                    new_index = costmap_->getIndex(neighbors[i][0], neighbors[i][1]);
                    ROS_INFO("Querying node (%d, %d), which has cost %d", neighbors[i][0], neighbors[i][1], cost);
                    ret = visited.insert(new_index);
                    if (ret.second && cost < lethal_threshold_)
                    {
                        queue.push(new_index);
                        wavefront_cost_[new_index] = wavefront_cost_[current_index] + 1;
                    }
                }
            }
        }

        mapped_ = true;
        ROS_INFO("Finished propagating the wave...");
    }

    void GlobalPlanner::markOverlappedAsVisited(const unsigned int x, const unsigned int y)
    {
        std::vector<costmap_2d::MapLocation> covered_cells;
        geometry_msgs::PoseStamped robot_position;
        costmap_ros_->getRobotPose(robot_position);
        auto dx = x - robot_position.pose.position.x;
        auto dy = y - robot_position.pose.position.y;
        auto footprint = costmap_ros_->getRobotFootprint();
        std::vector<costmap_2d::MapLocation> rob_location;
        for (int i=0; i < footprint.size(); i++)
        {
            costmap_2d::MapLocation location;
            costmap_->worldToMap(footprint[i].x + dx, footprint[i].y + dy, location.x, location.y);
            rob_location.push_back(location);
        }

        costmap_->polygonOutlineCells(rob_location, covered_cells);
        for (int i=0; i < covered_cells.size(); i++)
        {
            auto temp_index = costmap_->getIndex(covered_cells[i].x, covered_cells[i].y);
            visited_.insert(temp_index);
        }
    }

    // bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){        
    //     return true;
    // }

    bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){        
        // unsigned int current_x;
        // unsigned int current_y;
        // unsigned int current_index;
        geometry_msgs::PoseStamped robot_pose;
        // geometry_msgs::PoseStamped current_pose;
        costmap_2d::MapLocation map_location;
        visited_.clear();
        // tf::Stamped<tf::Pose> current_pose_tf;
        // tf::Stamped<tf::Pose> robot_pose_tf;
        // std::vector<geometry_msgs::Point> footprint;
        // std::pair<std::set<unsigned int>::iterator, bool> ret;

        costmap_ros_->getRobotPose(robot_pose);
        // current_index = costmap_->getIndex(current_x, current_y);
        costmap_->worldToMap(robot_pose.pose.position.x, robot_pose.pose.position.y, map_location.x, map_location.y);
        ROS_INFO("Found the robot at (%d, %d)", map_location.x, map_location.y);
        if (!mapped_){
            unsigned int goal_x;
            unsigned int goal_y;
            // costmap_2d::MapLocation goal_cell;
            costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y);
            ROS_INFO("Propagating wave from (%d, %d)", goal_x, goal_y);
            computeWavefront(goal_x, goal_y);
        }

        // plan.push_back(start);
        // auto footprint = costmap_ros_->getRobotFootprint();
        // float radius = 2*(robot_pose.pose.position.x - footprint[1].x);
        // ROS_INFO("Determined the radius of the robot as %f", radius);
        // auto dx_ = robot_pose.pose.position.x - 
        // tf::poseStampedMsgToTF(robot_pose, robot_pose_tf);
        // auto footprint = costmap_ros_->getRobotFootprint();
        // for (int i=0; i < footprint.size(); i++)
        // {
        //     tf::Point pt;
        //     tf::pointMsgToTF(footprint[i], pt);
        //     auto new_pt = pt * 
        // }
        std::vector<costmap_2d::MapLocation> map_coords;
        // markOverlappedAsVisited(current_x, current_y);
        plan.push_back(start);
        plan.push_back(robot_pose);
        map_coords.push_back(map_location);
        auto foo = costmap_->getIndex(map_location.x, map_location.y);
        visited_.insert(foo);
        while (true)
        {
            // unsigned int next_index;
            unsigned int cost = 0;
            unsigned int next_x;
            unsigned int next_y;
            unsigned int neighbors [4][2];
            auto current_x = map_coords.back().x;
            auto current_y = map_coords.back().y;

            neighbors[0][0] = current_x + 1;
            neighbors[0][1] = current_y;
            neighbors[1][0] = current_x - 1;
            neighbors[1][1] = current_y;
            neighbors[2][0] = current_x;
            neighbors[2][1] = current_y + 1;
            neighbors[3][0] = current_x;
            neighbors[3][1] = current_y - 1;

            for (int i=0; i < 4; i++)
            {
                // int neighbor_x;
                // int neighbor_y;

                if (neighbors[i][0] >= costmap_->getSizeInCellsX() || neighbors[i][0] < 0)
                {
                    continue;
                }
                if (neighbors[i][1] >= costmap_->getSizeInCellsY() || neighbors[i][1] < 0)
                {
                    continue;
                }
                // costmap_->worldToMapEnforceBounds(neighbors[i][0], neighbors[i][1], neighbor_x, neighbor_y);
                ROS_INFO("Testing neighbor at (%d, %d)", neighbors[i][0], neighbors[i][1]);
                auto temp_index = costmap_->getIndex(neighbors[i][0], neighbors[i][1]);
                auto it = wavefront_cost_.find(temp_index);
                auto v_it = visited_.find(temp_index);
                auto foo = costmap_->getCost(neighbors[i][0], neighbors[i][1]);
                ROS_INFO("%d", foo);
                if (v_it == visited_.end() && it != wavefront_cost_.end())
                {
                    auto weighted_cost = wavefront_cost_[temp_index];
                    ROS_INFO("Found cost %d", weighted_cost);
                    // auto weighted_cost = wavefront_cost_[temp_index] + costmap_->getCost(neighbors[i][0], neighbors[i][1]);
                    if (weighted_cost > cost)
                    {
                        cost = weighted_cost;
                        next_x = neighbors[i][0];
                        next_y = neighbors[i][1];
                        // next_index = temp_index;
                    }
                }
            }

            if (cost == 0)
            {
                ROS_INFO("Complete!");
                // plan.push_back(goal);

                return true;
            }
            else
            {
                double x;
                double y;
                // geometry_msgs::PoseStamped new_pose;
                ROS_INFO("Setting next point at (%d, %d)", next_x, next_y);
                // auto temp_index = costmap_->getIndex(next_x, next_y);
                // visited_.insert(temp_index);
                costmap_->mapToWorld(next_x, next_y, x, y);
                ROS_INFO("Setting next point at (%f, %f)", x, y);
                ROS_INFO("%d", (int)plan.size());
                geometry_msgs::PoseStamped current_pose;
                current_pose.header.frame_id = "map";
                current_pose.pose.position.x = x;
                current_pose.pose.position.y = y;
                if (plan.size() > 0){
                    auto dx = x - plan.back().pose.position.x;
                    auto dy = y - plan.back().pose.position.y;
                    auto theta = atan2(dy, dx);
                    auto new_orientation = tf::createQuaternionFromYaw(theta);
                    current_pose.pose.orientation.x = new_orientation.x();
                    current_pose.pose.orientation.y = new_orientation.y();
                    current_pose.pose.orientation.z = new_orientation.z();
                    current_pose.pose.orientation.w = new_orientation.w();                    
                }
                else {
                    current_pose.pose.orientation.x = robot_pose.pose.orientation.x;
                    current_pose.pose.orientation.y = robot_pose.pose.orientation.y;
                    current_pose.pose.orientation.z = robot_pose.pose.orientation.z;
                    current_pose.pose.orientation.w = robot_pose.pose.orientation.w;
                }
                
                costmap_2d::MapLocation temp_location;
                auto goo = costmap_->getIndex(next_x, next_y);
                visited_.insert(goo);
                temp_location.x = next_x;
                temp_location.y = next_y;
                map_coords.push_back(temp_location);
                plan.push_back(current_pose);
            }
        }
    }
 };