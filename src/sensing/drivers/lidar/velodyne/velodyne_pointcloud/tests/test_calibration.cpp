//
// C++ unit tests for calibration interface.
//

#include <gtest/gtest.h>

#include <ros/package.h>
#include <velodyne_pointcloud/calibration.h>
using namespace velodyne_pointcloud;

// global test data
std::string g_package_name("velodyne_pointcloud");
std::string g_package_path;

void init_global_data(void)
{
  g_package_path = ros::package::getPath(g_package_name);
}

///////////////////////////////////////////////////////////////
// Test cases
///////////////////////////////////////////////////////////////

TEST(Calibration, missing_file)
{
  Calibration calibration(false);
  calibration.read("./no_such_file.yaml");
  EXPECT_FALSE(calibration.initialized);
}

TEST(Calibration, vlp16)
{
  Calibration calibration(g_package_path + "/params/VLP16db.yaml", false);
  EXPECT_TRUE(calibration.initialized);
  ASSERT_EQ(calibration.num_lasers, 16);

  // check some values for the first laser:
  LaserCorrection laser = calibration.laser_corrections[0];
  EXPECT_FALSE(laser.two_pt_correction_available);
  EXPECT_FLOAT_EQ(laser.vert_correction, -0.2617993877991494);
  EXPECT_FLOAT_EQ(laser.horiz_offset_correction, 0.0);
  EXPECT_EQ(laser.max_intensity, 255);
  EXPECT_EQ(laser.min_intensity, 0);

  // check similar values for the last laser:
  laser = calibration.laser_corrections[15];
  EXPECT_FALSE(laser.two_pt_correction_available);
  EXPECT_FLOAT_EQ(laser.vert_correction, 0.2617993877991494);
  EXPECT_FLOAT_EQ(laser.horiz_offset_correction, 0.0);
  EXPECT_EQ(laser.max_intensity, 255);
  EXPECT_EQ(laser.min_intensity, 0);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  init_global_data();
  return RUN_ALL_TESTS();
}

