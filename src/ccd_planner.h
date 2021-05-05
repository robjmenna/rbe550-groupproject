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
        float alpha_;
        float radius = 0.3;
        int cell_radius = 1;
        unsigned int lethal_threshold_ = 150;
        std::set<int> visited_;
        bool mapped_ = false;
        std::map<int, int> wavefront_cost_;

        void getNeighbors(const unsigned int x, const unsigned int y, costmap_2d::MapLocation (&neighbors)[4]);
        void computeWavefront(const unsigned int start_x, const unsigned int start_y);
        void markOverlappedAsVisited(const unsigned int x, const unsigned int y,std::set<int>& visited);
        void search_dprime(const costmap_2d::MapLocation& start_location, std::set<int>& visited_sofar, std::vector<geometry_msgs::PoseStamped>& plan);
        void appendPose(const unsigned int x, const unsigned int y, std::vector<geometry_msgs::PoseStamped>& plan);
    };
};
#endif