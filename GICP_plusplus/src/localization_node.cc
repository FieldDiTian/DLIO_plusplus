/***********************************************************
 *                                                         *
 * Copyright (c)                                           *
 *                                                         *
 * The Verifiable & Control-Theoretic Robotics (VECTR) Lab *
 * University of California, Los Angeles                   *
 *                                                         *
 * Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez   *
 * Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu         *
 *                                                         *
 ***********************************************************/

#include "gicp_plusplus/localization.h"

#include <rclcpp/contexts/default_context.hpp>

int main(int argc, char** argv) {

  rclcpp::init(argc, argv);

  auto node = std::make_shared<gicp_plusplus::LocalizationNode>();
  node->start();

  // Drain the front synchronizer BEFORE the ROS context is invalidated.
  // Ctrl-C / SIGTERM (the bag-EOS teardown path) trigger rclcpp::shutdown()
  // while fronts may still be queued; pre-shutdown callbacks run before
  // rcl_shutdown, so rclcpp::ok() is still true and the worker PROCESSES the
  // run tail in order instead of counting it as shutdown_unprocessed (which
  // is what happens if the drain is left to the destructor).
  std::weak_ptr<gicp_plusplus::LocalizationNode> weak_node = node;
  auto context = rclcpp::contexts::get_global_default_context();
  auto pre_shutdown_handle = context->add_pre_shutdown_callback([weak_node]() {
    if (auto locked = weak_node.lock()) {
      locked->drainFrontSync();
    }
  });

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  int exit_code = 0;
  try {
    executor.spin();
  } catch (const std::exception& e) {
    // Legacy synchronous path: a strict-merge abort (require_all_aux past
    // budget) throws on an executor thread and lands here instead of
    // std::terminate. Report and exit nonzero after an orderly shutdown.
    RCLCPP_FATAL(node->get_logger(), "executor stopped by exception: %s", e.what());
    exit_code = 1;
  }

  rclcpp::shutdown();  // runs the pre-shutdown drain if not already run
  context->remove_pre_shutdown_callback(pre_shutdown_handle);

  // A pipeline exception on the synchronizer worker is a controlled shutdown
  // (no throw escapes the thread) — surface it in the exit code.
  if (node->syncFatal()) {
    exit_code = 1;
  }

  return exit_code;
}
