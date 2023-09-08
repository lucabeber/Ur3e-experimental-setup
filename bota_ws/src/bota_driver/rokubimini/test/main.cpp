/**
 * @authors     Mike Karamousadakis
 * @affiliation BOTA SYS A.G.
 * @brief       Main test file
 */

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

using ::testing::InitGoogleTest;

/* RUN TESTS */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
