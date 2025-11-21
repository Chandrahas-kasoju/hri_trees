    
#ifndef MY_ACTION_1_HPP_
#define MY_ACTION_1_HPP_

#include "behaviortree_cpp_v3/action_node.h"
#include <thread>
#include <mutex>
#include <chrono>
#include <atomic>

namespace hri_trees
{

class MyAsyncAction : public BT::StatefulActionNode
{
public:
  // Constructor
  MyAsyncAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config)
  {}

  // It's good practice to define a static method for the required ports
  static BT::PortsList providedPorts()
  {
    // This action will accept a parameter "duration_ms" from the XML
    return{ BT::InputPort<int>("duration_ms", 1000, "Time to wait (ms)") };
  }

  // This is called once when the node is first ticked.
  // It must return RUNNING if the action was started correctly.
  BT::NodeStatus onStart() override;

  // This is called periodically while the node is in the RUNNING state.
  // It should return SUCCESS or FAILURE to complete the action.
  // It can also return RUNNING if the action is not done yet.
  BT::NodeStatus onRunning() override;

  // This is called if the node is halted by the tree.
  // Halted means that the execution is aborted by a parent node.
  void onHalted() override;

private:
  // The long-running task will be executed in this thread
  std::thread worker_thread_;
  
  // This atomic variable is used to signal the result of the background task
  // to the main thread. It's atomic to ensure thread-safe access without a mutex.
  std::atomic<BT::NodeStatus> result_;

  // This atomic flag is used to signal the background thread to stop
  // if the node is halted.
  std::atomic<bool> halt_requested_;

};  // class MyAsyncAction
};  // namespace hri_trees


#endif  // MY_ACTION_1_HPP_

  