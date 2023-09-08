// std
#include <cmath>
#include <thread>

// gtest
#include <gtest/gtest.h>

// bota_worker
#include "bota_worker/Rate.hpp"

// Local and build server tolerances.
#define RATE_TEST_TOL_LOCAL 0.001
#define RATE_TEST_TOL_BUILD_SERVER 0.007

// Use the build server tolerance.
#define RATE_TEST_TOL RATE_TEST_TOL_BUILD_SERVER

/*!
 * Simulate some processing which takes a certain amount of time.
 * @param duration Processing duration in seconds.
 */
void doSomething(const double duration)
{
  std::this_thread::sleep_for(std::chrono::nanoseconds(static_cast<int64_t>(1e9 * duration)));
}

TEST(RateTest, Initialization)
{  // NOLINT
  const std::string name = "Test";
  const double time_step = 0.1;
  bota_worker::Rate rate(name, time_step);

  EXPECT_EQ(rate.getOptions().name_, name);
  EXPECT_EQ(rate.getOptions().timeStep_, time_step);
  EXPECT_EQ(rate.getOptions().maxTimeStepFactorWarning_, 1.0);
  EXPECT_EQ(rate.getOptions().maxTimeStepFactorError_, 10.0);
  EXPECT_EQ(rate.getOptions().enforceRate_, true);
  EXPECT_EQ(rate.getOptions().clockId_, CLOCK_MONOTONIC);
  EXPECT_EQ(rate.getNumTimeSteps(), 0u);
  EXPECT_EQ(rate.getNumWarnings(), 0u);
  EXPECT_EQ(rate.getNumErrors(), 0u);
  EXPECT_TRUE(std::isnan(rate.getAwakeTime()));
  EXPECT_TRUE(std::isnan(rate.getAwakeTimeMean()));
  EXPECT_TRUE(std::isnan(rate.getAwakeTimeStdDev()));
}

TEST(RateTest, Reset)
{  // NOLINT
  const double time_step = 0.1;
  const double processing_time = 0.05;

  // Run for one time step and reset.
  bota_worker::Rate rate("Test", time_step);
  doSomething(processing_time);
  rate.sleep();
  rate.reset();

  EXPECT_EQ(rate.getNumTimeSteps(), 0u);
  EXPECT_EQ(rate.getNumWarnings(), 0u);
  EXPECT_EQ(rate.getNumErrors(), 0u);
  EXPECT_TRUE(std::isnan(rate.getAwakeTime()));
  EXPECT_TRUE(std::isnan(rate.getAwakeTimeMean()));
  EXPECT_TRUE(std::isnan(rate.getAwakeTimeStdDev()));
}

TEST(RateTest, SleepWithEnforceRate)
{  // NOLINT
  const double time_step = 0.1;
  bota_worker::Rate rate("Test", time_step);
  rate.getOptions().enforceRate_ = true;

  timespec start{};
  timespec end{};
  const double processing_time = 0.05;
  std::vector<double> processing_times;
  std::vector<double> summed_step_times;

  // Test sleep() without processing.
  clock_gettime(CLOCK_MONOTONIC, &start);
  rate.reset();
  rate.sleep();
  clock_gettime(CLOCK_MONOTONIC, &end);
  EXPECT_NEAR(bota_worker::Rate::getDuration(start, end), time_step, RATE_TEST_TOL);

  // Test sleep() with processing additionally.
  clock_gettime(CLOCK_MONOTONIC, &start);
  rate.reset();
  doSomething(processing_time);
  rate.sleep();
  clock_gettime(CLOCK_MONOTONIC, &end);
  EXPECT_NEAR(bota_worker::Rate::getDuration(start, end), time_step, RATE_TEST_TOL);

  // Test sleep() with where one step takes too long, recovery within one step.
  processing_times = { 0.02, 0.02, 0.15, 0.02, 0.02 };
  summed_step_times = { 0.0, 0.1, 0.2, 0.35, 0.4, 0.5 };
  rate.reset();
  clock_gettime(CLOCK_MONOTONIC, &start);
  for (unsigned int i = 0; i < processing_times.size(); i++)
  {
    clock_gettime(CLOCK_MONOTONIC, &end);
    EXPECT_NEAR(bota_worker::Rate::getDuration(start, end), summed_step_times[i], 2.0 * RATE_TEST_TOL);
    doSomething(processing_times[i]);
    rate.sleep();
  }
  clock_gettime(CLOCK_MONOTONIC, &end);
  EXPECT_NEAR(bota_worker::Rate::getDuration(start, end), summed_step_times[processing_times.size()],
              2.0 * RATE_TEST_TOL);

  // Test sleep() with where one step takes too long, recovery within two steps.
  processing_times = { 0.02, 0.02, 0.19, 0.02, 0.02, 0.02 };
  summed_step_times = { 0.0, 0.1, 0.2, 0.39, 0.41, 0.5, 0.6 };
  rate.reset();
  clock_gettime(CLOCK_MONOTONIC, &start);
  for (unsigned int i = 0; i < processing_times.size(); i++)
  {
    clock_gettime(CLOCK_MONOTONIC, &end);
    EXPECT_NEAR(bota_worker::Rate::getDuration(start, end), summed_step_times[i], 2.0 * RATE_TEST_TOL);
    doSomething(processing_times[i]);
    rate.sleep();
  }
  clock_gettime(CLOCK_MONOTONIC, &end);
  EXPECT_NEAR(bota_worker::Rate::getDuration(start, end), summed_step_times[processing_times.size()],
              2.0 * RATE_TEST_TOL);

  // Test sleep() with where two steps take too long, recovery within one step.
  processing_times = { 0.02, 0.02, 0.12, 0.12, 0.02, 0.02 };
  summed_step_times = { 0.0, 0.1, 0.2, 0.32, 0.44, 0.5, 0.6 };
  rate.reset();
  clock_gettime(CLOCK_MONOTONIC, &start);
  for (unsigned int i = 0; i < processing_times.size(); i++)
  {
    clock_gettime(CLOCK_MONOTONIC, &end);
    EXPECT_NEAR(bota_worker::Rate::getDuration(start, end), summed_step_times[i], 2.0 * RATE_TEST_TOL);
    doSomething(processing_times[i]);
    rate.sleep();
  }
  clock_gettime(CLOCK_MONOTONIC, &end);
  EXPECT_NEAR(bota_worker::Rate::getDuration(start, end), summed_step_times[processing_times.size()],
              2.0 * RATE_TEST_TOL);

  // Test sleep() with where two steps take too long, recovery within two steps.
  processing_times = { 0.02, 0.02, 0.12, 0.12, 0.08, 0.02, 0.02 };
  summed_step_times = { 0.0, 0.1, 0.2, 0.32, 0.44, 0.52, 0.6, 0.7 };
  rate.reset();
  clock_gettime(CLOCK_MONOTONIC, &start);
  for (unsigned int i = 0; i < processing_times.size(); i++)
  {
    clock_gettime(CLOCK_MONOTONIC, &end);
    EXPECT_NEAR(bota_worker::Rate::getDuration(start, end), summed_step_times[i], 2.0 * RATE_TEST_TOL);
    doSomething(processing_times[i]);
    rate.sleep();
  }
  clock_gettime(CLOCK_MONOTONIC, &end);
  EXPECT_NEAR(bota_worker::Rate::getDuration(start, end), summed_step_times[processing_times.size()],
              2.0 * RATE_TEST_TOL);
}

TEST(RateTest, SleepWithoutEnforceRate)
{  // NOLINT
  const double time_step = 0.1;
  bota_worker::Rate rate("Test", time_step);
  rate.getOptions().enforceRate_ = false;

  timespec start{};
  timespec end{};
  const double processing_time = 0.05;
  std::vector<double> processing_times;
  std::vector<double> summed_step_times;

  // Test sleep() without processing.
  clock_gettime(CLOCK_MONOTONIC, &start);
  rate.reset();
  rate.sleep();
  clock_gettime(CLOCK_MONOTONIC, &end);
  EXPECT_NEAR(bota_worker::Rate::getDuration(start, end), time_step, RATE_TEST_TOL);

  // Test sleep() with processing.
  clock_gettime(CLOCK_MONOTONIC, &start);
  rate.reset();
  doSomething(processing_time);
  rate.sleep();
  clock_gettime(CLOCK_MONOTONIC, &end);
  EXPECT_NEAR(bota_worker::Rate::getDuration(start, end), time_step, RATE_TEST_TOL);

  // Test sleep() with where one step takes too long.
  processing_times = { 0.02, 0.02, 0.15, 0.02, 0.02 };
  summed_step_times = { 0.0, 0.1, 0.2, 0.35, 0.45, 0.55 };
  rate.reset();
  clock_gettime(CLOCK_MONOTONIC, &start);
  for (unsigned int i = 0; i < processing_times.size(); i++)
  {
    clock_gettime(CLOCK_MONOTONIC, &end);
    EXPECT_NEAR(bota_worker::Rate::getDuration(start, end), summed_step_times[i], 2.0 * RATE_TEST_TOL);
    doSomething(processing_times[i]);
    rate.sleep();
  }
  clock_gettime(CLOCK_MONOTONIC, &end);
  EXPECT_NEAR(bota_worker::Rate::getDuration(start, end), summed_step_times[processing_times.size()],
              2.0 * RATE_TEST_TOL);

  // Test sleep() with where two steps take too long.
  processing_times = { 0.02, 0.02, 0.12, 0.12, 0.02, 0.02 };
  summed_step_times = { 0.0, 0.1, 0.2, 0.32, 0.44, 0.54, 0.64 };
  rate.reset();
  clock_gettime(CLOCK_MONOTONIC, &start);
  for (unsigned int i = 0; i < processing_times.size(); i++)
  {
    clock_gettime(CLOCK_MONOTONIC, &end);
    EXPECT_NEAR(bota_worker::Rate::getDuration(start, end), summed_step_times[i], 4.0 * RATE_TEST_TOL);
    doSomething(processing_times[i]);
    rate.sleep();
  }
  clock_gettime(CLOCK_MONOTONIC, &end);
  EXPECT_NEAR(bota_worker::Rate::getDuration(start, end), summed_step_times[processing_times.size()],
              4.0 * RATE_TEST_TOL);
}

TEST(RateTest, WarningsAndErrors)
{  // NOLINT
  const double time_step = 0.1;
  bota_worker::Rate rate("Test", time_step);
  doSomething(0.5 * time_step);  // Ok
  rate.sleep();
  doSomething(2.0 * time_step);  // Warning
  rate.sleep();
  doSomething(3.0 * time_step);  // Warning
  rate.sleep();
  doSomething(11.0 * time_step);  // Error
  rate.sleep();

  EXPECT_EQ(rate.getNumTimeSteps(), 4u);
  EXPECT_EQ(rate.getNumWarnings(), 2u);
  EXPECT_EQ(rate.getNumErrors(), 1u);
}

TEST(RateTest, StatisticsWithEnforceRate)
{  // NOLINT
  const double time_step = 0.1;
  bota_worker::Rate rate("Test", time_step);
  rate.getOptions().enforceRate_ = true;

  // Test 1 time step.
  const double processing_time = 0.05;
  rate.reset();
  doSomething(processing_time);
  rate.sleep();

  EXPECT_EQ(rate.getNumTimeSteps(), 1u);
  EXPECT_NEAR(rate.getAwakeTime(), processing_time, 2.0 * RATE_TEST_TOL);
  EXPECT_NEAR(rate.getAwakeTimeMean(), processing_time, RATE_TEST_TOL);
  EXPECT_TRUE(std::isnan(rate.getAwakeTimeStdDev()));

  // Test 10 time steps with similar processing times.
  const unsigned int num_time_steps = 10;
  rate.reset();
  for (unsigned int i = 0; i < num_time_steps; i++)
  {
    doSomething(processing_time);
    rate.sleep();
  }

  EXPECT_EQ(rate.getNumTimeSteps(), num_time_steps);
  EXPECT_NEAR(rate.getAwakeTime(), processing_time, 2.0 * RATE_TEST_TOL);
  EXPECT_NEAR(rate.getAwakeTimeMean(), processing_time, RATE_TEST_TOL);
  EXPECT_LE(rate.getAwakeTimeStdDev(), RATE_TEST_TOL);
  EXPECT_FALSE(rate.getAwakeTimeStdDev() == 0.0);  // If it is 0.0 something is fishy.

  // Test 9 time steps with different processing times.
  const std::vector<double> processing_times = { 0.04, 0.02, 0.04, 0.07, 0.05, 0.05, 0.04, 0.09, 0.05 };
  rate.reset();
  for (double processing_time : processing_times)
  {
    doSomething(processing_time);
    rate.sleep();
  }

  EXPECT_EQ(rate.getNumTimeSteps(), processing_times.size());
  EXPECT_NEAR(rate.getAwakeTime(), *processing_times.rbegin(), 2.0 * RATE_TEST_TOL);
  EXPECT_NEAR(rate.getAwakeTimeMean(), 0.05, RATE_TEST_TOL);
  EXPECT_NEAR(rate.getAwakeTimeStdDev(), 0.02, RATE_TEST_TOL);

  // Test again with time step violation.
  rate.getOptions().timeStep_ = 0.035;
  rate.reset();
  for (double processing_time : processing_times)
  {
    doSomething(processing_time);
    rate.sleep();
  }

  EXPECT_EQ(rate.getNumTimeSteps(), processing_times.size());
  EXPECT_NEAR(rate.getAwakeTime(), *processing_times.rbegin(), 2.0 * RATE_TEST_TOL);
  EXPECT_NEAR(rate.getAwakeTimeMean(), 0.05, RATE_TEST_TOL);
  EXPECT_NEAR(rate.getAwakeTimeStdDev(), 0.02, RATE_TEST_TOL);
}

TEST(RateTest, StatisticsWithoutEnforceRate)
{  // NOLINT
  const double time_step = 0.1;
  bota_worker::Rate rate("Test", time_step);
  rate.getOptions().enforceRate_ = false;

  // Test 1 time step.
  const double processing_time = 0.05;
  rate.reset();
  doSomething(processing_time);
  rate.sleep();

  EXPECT_EQ(rate.getNumTimeSteps(), 1u);
  EXPECT_NEAR(rate.getAwakeTime(), processing_time, 2.0 * RATE_TEST_TOL);
  EXPECT_NEAR(rate.getAwakeTimeMean(), processing_time, RATE_TEST_TOL);
  EXPECT_TRUE(std::isnan(rate.getAwakeTimeStdDev()));

  // Test 10 time steps with similar processing times.
  const unsigned int num_time_steps = 10;
  rate.reset();
  for (unsigned int i = 0; i < num_time_steps; i++)
  {
    doSomething(processing_time);
    rate.sleep();
  }

  EXPECT_EQ(rate.getNumTimeSteps(), num_time_steps);
  EXPECT_NEAR(rate.getAwakeTime(), processing_time, 2.0 * RATE_TEST_TOL);
  EXPECT_NEAR(rate.getAwakeTimeMean(), processing_time, RATE_TEST_TOL);
  EXPECT_LE(rate.getAwakeTimeStdDev(), RATE_TEST_TOL);
  EXPECT_FALSE(rate.getAwakeTimeStdDev() == 0.0);  // If it is 0.0 something is fishy.

  // Test 9 time steps with different processing times.
  const std::vector<double> processing_times = { 0.04, 0.02, 0.04, 0.07, 0.05, 0.05, 0.04, 0.09, 0.05 };
  rate.reset();
  for (double processing_time : processing_times)
  {
    doSomething(processing_time);
    rate.sleep();
  }

  EXPECT_EQ(rate.getNumTimeSteps(), processing_times.size());
  EXPECT_NEAR(rate.getAwakeTime(), *processing_times.rbegin(), 2.0 * RATE_TEST_TOL);
  EXPECT_NEAR(rate.getAwakeTimeMean(), 0.05, RATE_TEST_TOL);
  EXPECT_NEAR(rate.getAwakeTimeStdDev(), 0.02, RATE_TEST_TOL);

  // Test again with time step violation.
  rate.getOptions().timeStep_ = 0.035;
  rate.reset();
  for (double processing_time : processing_times)
  {
    doSomething(processing_time);
    rate.sleep();
  }

  EXPECT_EQ(rate.getNumTimeSteps(), processing_times.size());
  EXPECT_NEAR(rate.getAwakeTime(), *processing_times.rbegin(), 2.0 * RATE_TEST_TOL);
  EXPECT_NEAR(rate.getAwakeTimeMean(), 0.05, RATE_TEST_TOL);
  EXPECT_NEAR(rate.getAwakeTimeStdDev(), 0.02, RATE_TEST_TOL);
}
