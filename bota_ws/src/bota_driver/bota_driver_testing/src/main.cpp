/**
 * @authors     Mike Karamousadakis
 * @affiliation BOTA SYS A.G.
 * @brief       Main test file
 */

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

using ::testing::InitGoogleTest;

int main(int argc, char** argv)
{
  InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}