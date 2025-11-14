#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components_msgs/srv/load_node.hpp>

using LoadSrv = rclcpp_components_msgs::srv::LoadNode;
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("load_composable_node_client");

    auto client = node->create_client<LoadSrv>("load_composable_node");

    if (!client->wait_for_service(std::chrono::seconds(3))) {
        RCLCPP_ERROR(node->get_logger(), "Service not available");
        return 1;
    }

    auto req = std::make_shared<LoadSrv::Request>();
    req->package_name = "composition";
    req->plugin_name = "composition::Talker";
    req->node_name = "talker";
    req->node_namespace = "/demo";
    req->log_level = 0;

    auto future = client->async_send_request(req);
    auto status = future.wait_for(std::chrono::seconds(5));

    if (status == std::future_status::ready) {
        auto res = future.get();
        if (res->success) {
            RCLCPP_INFO(node->get_logger(), "Loaded node: %s (ID=%lu)",
                        res->full_node_name.c_str(), res->unique_id);
        } else {
            RCLCPP_ERROR(node->get_logger(), "Error: %s", res->error_message.c_str());
        }
    }

    rclcpp::shutdown();
    return 0;
}
