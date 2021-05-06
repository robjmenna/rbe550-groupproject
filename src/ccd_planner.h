 /** include the libraries you need in your planner here */
 /** for global path planner interface */
 #include <ros/ros.h>
 #include <costmap_2d/costmap_2d_ros.h>
 #include <costmap_2d/costmap_2d.h>
 #include <nav_core/base_global_planner.h>
 #include <geometry_msgs/PoseStamped.h>
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
        double alpha_ = 0.5;
        // float radius = 0.3;
        int cell_radius = 1;
        unsigned int lethal_threshold_ = 100;
        int clean_threshold = 4;
        std::vector<std::vector<unsigned int>> wavefront_map_;
        nav_msgs::OccupancyGrid coverage_grid;
        // std::set<int> visited_;
        // geometry_msgs::Pose last_goal;
        // bool mapped_ = false;
        // std::map<int, double> wavefront_cost_;
        // std::vector<geometry_msgs::PoseStamped> last_plan;

        void planPath(const costmap_2d::MapLocation& start, const costmap_2d::MapLocation& goal, std::vector<geometry_msgs::PoseStamped>& plan,std::vector<std::vector<unsigned int>> wavefront_map);
        void getNeighbors(const unsigned int x, const unsigned int y, costmap_2d::MapLocation (&neighbors)[4]);
        void getNeighborsFine(const unsigned int x, const unsigned int y, costmap_2d::MapLocation (&neighbors)[4]);
        void computeWavefront(const costmap_2d::MapLocation& goal, std::vector<std::vector<unsigned int>>& wavefront_map);
        void markOverlappedAsVisited(const costmap_2d::MapLocation& pos, std::set<int>& visited);
        void search_dprime(const costmap_2d::MapLocation& start_location, std::set<int>& visited_sofar, std::vector<geometry_msgs::PoseStamped>& plan);
        void appendPose(const costmap_2d::MapLocation& location, std::vector<geometry_msgs::PoseStamped>& plan);
        void GlobalPlanner::setCoverageGrid(const nav_msgs::OccupancyGrid& grid);
    };
};
#endif