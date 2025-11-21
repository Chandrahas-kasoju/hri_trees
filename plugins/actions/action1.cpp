#include "hri_trees/action1.hpp"
namespace hri_trees
{
BT::NodeStatus MyAsyncAction::onStart()
{
  // Get the duration from the input port
  int duration_ms;
  if (!getInput<int>("duration_ms", duration_ms)) {
    // If the port is not provided, throw an error
    throw BT::RuntimeError("Missing required input [duration_ms]");
  }

  printf("[MyAsyncAction] Starting action with duration: %d ms\n", duration_ms);

  // Reset the flags
  halt_requested_ = false;
  result_ = BT::NodeStatus::RUNNING;

  // Launch the long-running task in a separate thread
  worker_thread_ = std::thread([this, duration_ms]() {
    // This lambda function is the body of the worker thread.

    printf("[Worker Thread] Started.\n");
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // The main loop of the task
    while (std::chrono::high_resolution_clock::now() - start_time < std::chrono::milliseconds(duration_ms))
    {
      // --- IMPORTANT ---
      // This is where you check if the action has been halted.
      // If your task is a long loop, you must check this flag periodically.
      // If your task is a single blocking call, onHalted() becomes more complex.
      if (halt_requested_) {
        printf("[Worker Thread] Halt requested. Aborting.\n");
        // We don't set a result because the node was halted, not completed.
        return;
      }
      // Sleep for a short time to simulate work and not spin the CPU
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // If the thread was not halted, it has completed its task.
    // We can set the result.
    printf("[Worker Thread] Task completed.\n");
    
    // Simulate a random success or failure
    if (rand() % 2 == 0) {
        result_ = BT::NodeStatus::SUCCESS;
    } else {
        result_ = BT::NodeStatus::FAILURE;
    }
  });

  // The onStart method returns RUNNING immediately.
  // The tree will then periodically call onRunning().
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MyAsyncAction::onRunning()
{
  // The onRunning function simply checks the atomic variable 'result_'.
  // It will be called periodically by the Behavior Tree ticker.
  // The value of 'result_' is set by the worker thread once the task is complete.
  return result_.load();
}

void MyAsyncAction::onHalted()
{
  // This method is called by the Behavior Tree if the node is aborted.
  // For example, if a parent Fallback node has a child that returns SUCCESS.
  
  printf("[MyAsyncAction] Halted.\n");
  
  // We signal the worker thread to stop its execution.
  halt_requested_ = true;
  
  // It's important to join the thread to make sure it finishes its cleanup
  // before the node is destroyed.
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}
};  // namespace hri_trees