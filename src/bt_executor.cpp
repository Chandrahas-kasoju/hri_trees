#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include <memory>

// --- IMPORTANT: Include the headers for your custom BT nodes ---
// Make sure the path matches your package structure.
#include "hri_trees/action1.hpp" 
// #include "hri_trees/my_async_action.hpp" // Uncomment if you create this one too

// We are putting our node in the same namespace as our package
namespace hri_trees
{

// Using a shorter alias for the lifecycle node's return type
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * @class BtExecutorNode
 * @brief A lifecycle node that loads, manages, and executes a Behavior Tree.
 */
class BtExecutorNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  /**
   * @brief Constructor for the BtExecutorNode.
   * @param options Node options for the lifecycle node.
   */
  explicit BtExecutorNode(const rclcpp::NodeOptions & options)
    : rclcpp_lifecycle::LifecycleNode("bt_executor", options)
  {
    RCLCPP_INFO(get_logger(), "BtExecutorNode created.");
  }

  //--- Lifecycle Callback Implementations ---//

  /**
   * @brief on_configure: Called when the node enters the 'configuring' state.
   * This is where we load the BT, register plugins, and set up resources.
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "Configuring Behavior Tree Executor...");

    // Declare and get the path to the behavior tree XML file from a ROS parameter
    this->declare_parameter<std::string>("bt_xml_file", "");
    std::string bt_xml_file = this->get_parameter("bt_xml_file").as_string();

    if (bt_xml_file.empty()) {
      RCLCPP_ERROR(get_logger(), "'bt_xml_file' parameter not set or empty. Configuration failed.");
      return CallbackReturn::FAILURE;
    }

    RCLCPP_INFO(get_logger(), "Loading BT XML file: %s", bt_xml_file.c_str());

    // Create the BehaviorTreeFactory
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();

    // --- IMPORTANT: Register your custom nodes here! ---
    // The first argument is the name you will use for the node in your XML file.
    // The second argument is the C++ class name.
    factory_->registerNodeType<hri_trees::MyAsyncAction>("Action1");
    // factory_->registerNodeType<MyAsyncAction>("MyAsyncAction"); // Uncomment if you add more

    // Create the behavior tree from the XML file
    // Note: The BT::Tree object has a deleted copy constructor, so we must
    // store it in a pointer. A unique_ptr is a good choice as this class is the sole owner.
    tree_ = std::make_unique<BT::Tree>(factory_->createTreeFromFile(bt_xml_file));

    // Add loggers for debugging and visualization
    // StdCoutLogger prints state changes to the console
    logger_cout_ = std::make_unique<BT::StdCoutLogger>(*tree_);
    // PublisherZMQ allows you to connect Groot to visualize the tree's execution
    publisher_zmq_ = std::make_unique<BT::PublisherZMQ>(*tree_);

    RCLCPP_INFO(get_logger(), "Configuration successful.");
    return CallbackReturn::SUCCESS;
  }

  /**
   * @brief on_activate: Called when the node enters the 'active' state.
   * This is where we start the execution of the Behavior Tree.
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "Activating Behavior Tree Executor.");

    // The BT is ticked at a fixed rate by a ROS timer.
    const auto timer_period = std::chrono::milliseconds(100); // 10 Hz
    timer_ = this->create_wall_timer(timer_period, std::bind(&BtExecutorNode::tick_tree, this));

    RCLCPP_INFO(get_logger(), "Activation successful. Tree is now running.");
    return CallbackReturn::SUCCESS;
  }

  /**
   * @brief on_deactivate: Called when the node enters the 'inactive' state.
   * This is where we stop the execution of the Behavior Tree.
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "Deactivating Behavior Tree Executor.");
    
    // Cancel the timer to stop ticking the tree
    timer_->cancel();

    RCLCPP_INFO(get_logger(), "Deactivation successful. Tree is stopped.");
    return CallbackReturn::SUCCESS;
  }

  /**
   * @brief on_cleanup: Called when the node enters the 'unconfigured' state.
   * This is where we release all resources.
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "Cleaning up Behavior Tree Executor...");

    // Release all resources in reverse order of creation
    timer_.reset();
    publisher_zmq_.reset();
    logger_cout_.reset();
    tree_.reset();
    factory_.reset();
    
    RCLCPP_INFO(get_logger(), "Cleanup successful.");
    return CallbackReturn::SUCCESS;
  }

  /**
   * @brief on_shutdown: Called when the node is about to be destroyed.
   */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "Shutting down Behavior Tree Executor.");
    return CallbackReturn::SUCCESS;
  }

private:
  /**
   * @brief The main execution loop for the Behavior Tree, called by the timer.
   */
  void tick_tree()
  {
    BT::NodeStatus status = tree_->tickRoot();

    // If the tree returns a final state (SUCCESS or FAILURE), it has finished.
    // In this case, we can automatically deactivate the node to stop ticking.
    if (status != BT::NodeStatus::RUNNING)
    {
      RCLCPP_INFO(
        get_logger(),
        "Behavior Tree execution finished with status: %s. Deactivating node.",
        BT::toStr(status).c_str());
      this->deactivate();
    }
  }

  // --- Member Variables ---

  // The factory for creating the behavior tree
  std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  
  // The behavior tree object itself
  std::unique_ptr<BT::Tree> tree_;

  // Loggers for visualization and debugging
  std::unique_ptr<BT::StdCoutLogger> logger_cout_;
  std::unique_ptr<BT::PublisherZMQ> publisher_zmq_;
  
  // The ROS timer that ticks the tree
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace hri_trees

//--- The main function ---
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  // You must use a SingleThreadedExecutor for lifecycle nodes.
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::NodeOptions options;
  
  // Create an instance of the BtExecutorNode
  auto bt_executor_node = std::make_shared<hri_trees::BtExecutorNode>(options);
  
  // Add the node to the executor so its callbacks can be processed
  executor.add_node(bt_executor_node->get_node_base_interface());
  
  // Spin the executor to start processing events
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}