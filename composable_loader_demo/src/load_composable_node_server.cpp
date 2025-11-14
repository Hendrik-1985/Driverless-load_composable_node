#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/component_manager.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/srv/load_node.hpp>

using LoadNodeSrv = rclcpp_components::srv::LoadNode;
class ComposableLoaderServer : public rclcpp::Node
{
public:
    ComposableLoaderServer()
        : Node("load_composable_node_server")
    {
        
        component_manager_ =
            std::make_shared<rclcpp_components::ComponentManager>(
                rclcpp::Executor::WeakPtr{},        
                "component_manager",
                rclcpp::NodeOptions()
            );

        
        this->get_node_base_interface()->get_executor()->add_node(component_manager_);

        // pass on Service 
        service_ = this->create_service<LoadSrv>(
            "load_composable_node",
            std::bind(&ComposableLoaderServer::handle_load, this,
                      std::placeholders::_1, std::placeholders::_2)
        );
    }

private:
    rclcpp_components::ComponentManager::SharedPtr component_manager_;
    rclcpp::Service<LoadSrv>::SharedPtr service_;

    void handle_load(
        const std::shared_ptr<LoadSrv::Request> req,
        std::shared_ptr<LoadSrv::Response> res)
    {
        RCLCPP_INFO(this->get_logger(),
                    "Loading component: %s/%s",
                    req->package_name.c_str(),
                    req->plugin_name.c_str());

        // Calling the original ComponentManager service
        component_manager_->load_node(req, res);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto server = std::make_shared<ComposableLoaderServer>();
    executor->add_node(server);
    executor->spin();

    rclcpp::shutdown();
    return 0;
}
