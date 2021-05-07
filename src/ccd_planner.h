 /** include the libraries you need in your planner here */
 /** for global path planner interface */
 #include <ros/ros.h>
 #include <costmap_2d/costmap_2d_ros.h>
 #include <costmap_2d/costmap_2d.h>
 #include <nav_core/base_global_planner.h>
 #include <geometry_msgs/PoseStamped.h>
 #include <nav_msgs/OccupancyGrid.h>
 #include <angles/angles.h>
 #include <base_local_planner/world_model.h>
 #include <base_local_planner/costmap_model.h>
 #include <tf/tf.h>
 #include <math.h>

 using std::string;

 #ifndef GLOBAL_PLANNER_CPP
 #define GLOBAL_PLANNER_CPP

namespace ccd_planner {

    class GlobalPlanner : public nav_core::BaseGlobalPlanner {
    public:

        GlobalPlanner();
        GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        /** overridden classes from interface nav_core::BaseGlobalPlanner **/
        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
        bool makePlan(const geometry_msgs::PoseStamped& start,
                    const geometry_msgs::PoseStamped& goal,
                    std::vector<geometry_msgs::PoseStamped>& plan
                    );
    protected:
        costmap_2d::Costmap2D* costmap_;
        costmap_2d::Costmap2DROS* costmap_ros_;
        bool initialized_ = false;
        double alpha = 0.5;
        double beta = 0.5;
        double radius = 0.1;
        double resolution = 0.05;
        int max_path_length = 50;
        // int cell_radius = 1;
        int lethal_threshold_ = 150;
        unsigned int max_wave_cost=0;
        int clean_threshold = 4;
        std::vector<std::vector<unsigned int>> wavefront_map_;
        costmap_2d::MapLocation last_goal;
        nav_msgs::OccupancyGrid coverage_grid;

        costmap_2d::MapLocation last_location_;
        
        // std::set<int> visited_;
        // geometry_msgs::Pose last_goal;
        // bool mapped_ = false;
        // std::map<int, double> wavefront_cost_;
        // std::vector<geometry_msgs::PoseStamped> last_plan;

        void planPath(const costmap_2d::MapLocation& start, const costmap_2d::MapLocation& goal, std::vector<geometry_msgs::PoseStamped>& plan,std::vector<std::vector<unsigned int>> wavefront_map);
        void getNeighbors(const unsigned int x, const unsigned int y, const nav_msgs::OccupancyGridConstPtr clean_grid, std::set<int>& visited_sofar, std::vector<costmap_2d::MapLocation>& neighbors);
        void getNeighborsFine(const unsigned int x, const unsigned int y, std::vector<costmap_2d::MapLocation>& neighbors);
        void computeWavefront(const costmap_2d::MapLocation& goal, std::vector<std::vector<unsigned int>>& wavefront_map);
        void markOverlappedAsVisited(const costmap_2d::MapLocation& pos, std::set<int>& visited);
        void search_dprime(const costmap_2d::MapLocation& start_location, std::set<int>& visited_sofar, std::vector<geometry_msgs::PoseStamped>& plan, nav_msgs::OccupancyGridConstPtr clean_grid);
        void appendPose(const costmap_2d::MapLocation& location, std::vector<geometry_msgs::PoseStamped>& plan);
    };
};
#endif