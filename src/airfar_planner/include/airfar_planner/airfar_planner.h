#ifndef airfar_planner_H
#define airfar_planner_H

#include "utility.h"
#include "dynamic_graph.h"
#include "contour_detector.h"
#include "contour_graph.h"
#include "graph_planner.h"
#include "map_handler.h"
#include "planner_visualizer.h"
#include "scan_handler.h"
#include "graph_msger.h"


struct DPMasterParams {
    DPMasterParams() = default;
    float robot_dim; 
    float vehicle_height;
    float voxel_dim;
    float sensor_range;
    float waypoint_project_dist;
    std::string world_frame;
    float layer_resolution;
    int neighbor_layers;
    float main_run_freq;
    float viz_ratio;
    bool is_inter_navpoint;
    bool is_simulation;
    bool is_trajectory_edge;
    bool is_attempt_autoswitch;
};

class DPMaster {
public:
    DPMaster();
    ~DPMaster() = default;

    void Init(); // Node initialization
    rclcpp::Node::SharedPtr GetNode() { return nh_; }
    rclcpp::Node::SharedPtr GetControlNode() { return control_nh_; }
    void RunOnce();
    float MainRunFrequency() const { return master_params_.main_run_freq; }

private:
    rclcpp::Node::SharedPtr nh_;
    rclcpp::Node::SharedPtr control_nh_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_graph_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_command_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr terrain_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr terrain_local_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr goal_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr vertices_PCL_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obs_world_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr new_PCL_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dynamic_obs_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr surround_free_debug_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr surround_obs_debug_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_grid_debug_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pc_debug_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr mapping_time_pub_;
    rclcpp::TimerBase::SharedPtr main_event_;

    Point3D robot_pos_, robot_heading_, nav_heading_, nav_goal_;

    bool is_robot_stop_, is_new_iter_, is_reset_env_;

    geometry_msgs::msg::PointStamped goal_waypoint_stamped_;

    bool is_cloud_init_, is_scan_init_, is_odom_init_, is_planner_running_;
    bool is_goal_update_, is_dyobs_update_, is_graph_init_;

    std::vector<int> cur_layer_idxs_;

    PointCloudPtr new_vertices_ptr_;
    PointCloudPtr temp_obs_ptr_;
    PointCloudPtr temp_free_ptr_;
    PointCloudPtr temp_cloud_ptr_;
    PointCloudPtr local_terrian_ptr_;
    PointCloudPtr scan_grid_ptr_;

    NavNodePtr odom_node_ptr_ = NULL;
    NavNodePtr goal_node_ptr_ = NULL;
    NodePtrStack new_nodes_;
    NodePtrStack nav_graph_;
    NodePtrStack wide_nav_graph_;
    NodePtrStack clear_nodes_;

    CTNodeStack new_ctnodes_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    /* module objects */
    ContourDetector contour_detector_;
    DynamicGraph graph_manager_;
    DPVisualizer planner_viz_;
    GraphPlanner graph_planner_;
    ContourGraph contour_graph_;
    MapHandler map_handler_;
    ScanHandler scan_handler_;
    GraphMsger graph_msger_;

    /* ROS Params */
    DPMasterParams      master_params_;
    ContourDetectParams cdetect_params_;
    DynamicGraphParams  graph_params_;
    GraphPlannerParams  gp_params_;
    ContourGraphParams  cg_params_;
    MapHandlerParams    map_params_;
    ScanHandlerParams   scan_params_;
    GraphMsgerParams    msger_parmas_;
    
    void LoadROSParams();

    void ResetEnvironmentAndGraph();
    void MainLoopCallBack();

    void PrcocessCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pc,
                       const PointCloudPtr& cloudOut,
                       const bool& is_crop_cloud);

    Point3D ProjectNavWaypoint(const Point3D& nav_waypoint, const Point3D& last_waypoint);

    /* Callback Functions */
    void OdomCallBack(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
    void TerrainCallBack(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pc);
    void TerrainLocalCallBack(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pc);

    inline void ResetGraphCallBack(const std_msgs::msg::Empty::ConstSharedPtr msg) {
        (void)msg;
        is_reset_env_ = true;
    }

    inline void JoyCommandCallBack(const sensor_msgs::msg::Joy::ConstSharedPtr msg) {
        if (msg->buttons[4] > 0.5) {
            is_reset_env_ = true;
        }
    }

    void ScanCallBack(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pc);
    void WaypointCallBack(const geometry_msgs::msg::PointStamped::ConstSharedPtr msg);
    void GoalPoseCallBack(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);

    void ExtractDynamicObsFromScan(const PointCloudPtr& scanCloudIn, 
                                   const PointCloudPtr& obsCloudIn, 
                                   const PointCloudPtr& dyObsCloudOut);

    /* define inline functions */
    inline bool PreconditionCheck() {
        if (is_cloud_init_ && is_odom_init_) {
            return true;
        }
        return false;
    }
    inline void ClearTempMemory() {
        new_vertices_ptr_->clear();
        new_nodes_.clear();
        nav_graph_.clear();
        clear_nodes_.clear();
        new_ctnodes_.clear();
        wide_nav_graph_.clear();
    }

    inline void ResetInternalValues() {
        odom_node_ptr_ = NULL; 
        goal_node_ptr_ = NULL;
        is_cloud_init_      = false; 
        is_odom_init_       = false; 
        is_scan_init_       = false;
        is_planner_running_ = false; 
        is_goal_update_     = false;
        is_dyobs_update_    = false;  
        is_graph_init_      = false;
        is_robot_stop_      = false; 
        is_new_iter_        = false;
        ClearTempMemory();
    }
};

#endif
