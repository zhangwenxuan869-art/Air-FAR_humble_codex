/*
 * Dynamic Route Planner
 * Copyright (C) 2021 Fan Yang - All rights reserved
 * fanyang2@andrew.cmu.edu,   
 */



#include "airfar_planner/graph_msger.h"

/***************************************************************************************/

void GraphMsger::Init(const rclcpp::Node::SharedPtr nh, const GraphMsgerParams& params) {
    nh_ = nh;
    gm_params_ = params;
    graph_pub_ = nh_->create_publisher<nav_graph_msg::Graph>("/planner_nav_graph", 5);
    request_graph_service_ = nh_->create_service<std_srvs::srv::Trigger>("/request_nav_graph", std::bind(&GraphMsger::RequestGraphService, this, std::placeholders::_1, std::placeholders::_2));

    global_graph_.clear();
    nav_graph_.nodes.clear();
}

void GraphMsger::EncodeGraph(const NodePtrStack& graphIn, nav_graph_msg::Graph& graphOut) {
    graphOut.nodes.clear();
    const std::string frame_id = graphOut.header.frame_id;
    for (const auto& node_ptr : graphIn) {
        if (node_ptr->connect_nodes.empty() || !IsEncodeType(node_ptr)) continue;
        nav_graph_msg::Node msg_node;
        msg_node.header.frame_id = frame_id;
        msg_node.position.x = node_ptr->position.x;
        msg_node.position.y = node_ptr->position.y;
        msg_node.position.z = node_ptr->position.z;
        msg_node.id = node_ptr->id;
        msg_node.connect_nodes.clear();
        for (const auto& cnode_ptr : node_ptr->connect_nodes) {
            if (!IsEncodeType(cnode_ptr)) continue;
            msg_node.connect_nodes.push_back(cnode_ptr->id);
        }
        graphOut.nodes.push_back(msg_node);
    }
    graphOut.size = graphOut.nodes.size();
}

void GraphMsger::RequestGraphService(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                     std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    (void)req;
    if (global_graph_.empty()) {
        res->success = false;
        res->message = "Global graph is empty.";
        return;
    }
    nav_graph_.header.frame_id = gm_params_.frame_id;
    nav_graph_.robot_id = gm_params_.robot_id;
    this->EncodeGraph(global_graph_, nav_graph_);
    graph_pub_->publish(nav_graph_);
    res->message = "Global graph has been shared.";
    res->success = true;
    ROS_WARN("GM: Graph of robot %d has been published, total number of nodes: %d", gm_params_.robot_id, nav_graph_.size);
}
