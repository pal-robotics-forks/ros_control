///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2013, PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of hiDOF, Inc. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/// \author Adolfo Rodriguez Tsouroukdissian

#include <string>
#include <gtest/gtest.h>
#include <ros/console.h>
#include <hardware_interface/actuator_command_interface.h>

using std::string;
using namespace hardware_interface;

TEST(ActuatorCommandHandleTest, HandleConstruction)
{
  string name = "name1";
  double pos, vel, eff;
  double cmd;
  std::vector<double> pids_good(3, std::numeric_limits<double>::quiet_NaN());
  std::vector<double> pids_bad_1(2, std::numeric_limits<double>::quiet_NaN());
  std::vector<double> pids_bad_2(4, std::numeric_limits<double>::quiet_NaN());
  double ff_term;
  EXPECT_NO_THROW(ActuatorHandle tmp(ActuatorStateHandle(name, &pos, &vel, &eff), &cmd));
  EXPECT_THROW(ActuatorHandle tmp(ActuatorStateHandle(name, &pos, &vel, &eff), 0), HardwareInterfaceException);
  EXPECT_THROW(ActuatorHandle tmp(ActuatorStateHandle(name, &pos, &vel, &eff), &cmd, &pids_bad_1), HardwareInterfaceException);
  EXPECT_THROW(ActuatorHandle tmp(ActuatorStateHandle(name, &pos, &vel, &eff), &cmd, &pids_bad_2), HardwareInterfaceException);
  EXPECT_THROW(ActuatorHandle tmp(ActuatorStateHandle(name, &pos, &vel, &eff), &cmd, &pids_bad_2, &ff_term), HardwareInterfaceException);
  EXPECT_NO_THROW(ActuatorHandle tmp(ActuatorStateHandle(name, &pos, &vel, &eff), &cmd, &pids_good, &ff_term));
  // We don't throw exception in case the FF term info is not set
  EXPECT_NO_THROW(ActuatorHandle tmp(ActuatorStateHandle(name, &pos, &vel, &eff), &cmd, &pids_good, 0));

  // Print error messages
  // Requires manual output inspection, but exception message should be descriptive
  try {ActuatorHandle(ActuatorStateHandle(name, &pos, &vel, &eff), 0);}
  catch(const HardwareInterfaceException& e) {ROS_ERROR_STREAM(e.what());}
}

#ifndef NDEBUG // NOTE: This test validates assertion triggering, hence only gets compiled in debug mode
TEST(ActuatorStateHandleTest, AssertionTriggering)
{
  ActuatorHandle h;

  // Data with invalid pointers should trigger an assertion
  EXPECT_DEATH(h.getPosition(),   ".*");
  EXPECT_DEATH(h.getVelocity(),   ".*");
  EXPECT_DEATH(h.getEffort(),     ".*");
  EXPECT_DEATH(h.getCommand(),    ".*");
  EXPECT_DEATH(h.setCommand(1.0), ".*");
  EXPECT_DEATH(h.setPIDGains(1.0, 0.001, 0.1), ".*");
  EXPECT_DEATH(h.getPIDGains(), ".*");
  EXPECT_DEATH(h.setFFTerm(1.0), ".*");
  EXPECT_DEATH(h.getFFTerm(), ".*");
  EXPECT_FALSE(h.getFFTermConstPtr());
  EXPECT_FALSE(h.getPIDGainsConstPtr());
}
#endif // NDEBUG

class ActuatorCommandInterfaceTest : public ::testing::Test
{
public:
  ActuatorCommandInterfaceTest()
    : pos1(1.0), vel1(2.0), eff1(3.0), cmd1(0.0),
      pos2(4.0), vel2(5.0), eff2(6.0), cmd2(0.0),
      pids(3, std::numeric_limits<double>::quiet_NaN()),
      ff_term(std::numeric_limits<double>::quiet_NaN()),
      name1("name_1"),
      name2("name_2"),
      hs1(name1, &pos1, &vel1, &eff1),
      hs2(name2, &pos2, &vel2, &eff2),
      hc1(hs1, &cmd1, &pids, &ff_term),
      hc2(hs2, &cmd2)
  {}

protected:
  double pos1, vel1, eff1, cmd1;
  double pos2, vel2, eff2, cmd2;
  std::vector<double> pids;
  double ff_term;
  string name1;
  string name2;
  ActuatorStateHandle hs1, hs2;
  ActuatorHandle hc1, hc2;
};

TEST_F(ActuatorCommandInterfaceTest, ExcerciseApi)
{
  ActuatorCommandInterface iface;
  iface.registerHandle(hc1);
  iface.registerHandle(hc2);

  // Get handles
  EXPECT_NO_THROW(iface.getHandle(name1));
  EXPECT_NO_THROW(iface.getHandle(name2));

  ActuatorHandle hc1_tmp = iface.getHandle(name1);
  EXPECT_EQ(name1, hc1_tmp.getName());
  EXPECT_DOUBLE_EQ(pos1, hc1_tmp.getPosition());
  EXPECT_DOUBLE_EQ(vel1, hc1_tmp.getVelocity());
  EXPECT_DOUBLE_EQ(eff1, hc1_tmp.getEffort());
  EXPECT_DOUBLE_EQ(cmd1, hc1_tmp.getCommand());
  EXPECT_EQ(&pos1, hc1_tmp.getPositionPtr());
  EXPECT_EQ(&vel1, hc1_tmp.getVelocityPtr());
  EXPECT_EQ(&eff1, hc1_tmp.getEffortPtr());
  EXPECT_EQ(&cmd1, hc1_tmp.getCommandPtr());

  const double new_cmd_1 = -1.0;
  hc1_tmp.setCommand(new_cmd_1);
  EXPECT_DOUBLE_EQ(new_cmd_1, hc1_tmp.getCommand());

  // By default it is zero
  EXPECT_TRUE(std::isnan(hc1_tmp.getFFTerm()));
  const double new_ff_gain = 10.0;
  hc1_tmp.setFFTerm(new_ff_gain);
  EXPECT_DOUBLE_EQ(new_ff_gain, *hc1_tmp.getFFTermConstPtr());

  const std::vector<double>* pid_gains = hc1_tmp.getPIDGainsConstPtr();
  // Default values of the gains
  EXPECT_EQ(3, (*pid_gains).size());
  EXPECT_TRUE(std::isnan((*pid_gains)[0]));
  EXPECT_TRUE(std::isnan((*pid_gains)[1]));
  EXPECT_TRUE(std::isnan((*pid_gains)[2]));
  // Now change the values of the gains
  const double new_p_gain = 1000.0;
  const double new_i_gain = 1.0;
  const double new_d_gain = 10.0;
  hc1_tmp.setPIDGains(new_p_gain, new_i_gain, new_d_gain);
  EXPECT_DOUBLE_EQ(new_p_gain, (*pid_gains)[0]);
  EXPECT_DOUBLE_EQ(new_i_gain, (*pid_gains)[1]);
  EXPECT_DOUBLE_EQ(new_d_gain, (*pid_gains)[2]);
  // Test the same with other methods
  EXPECT_DOUBLE_EQ(new_p_gain, hc1_tmp.getPIDGains().p_gain_);
  EXPECT_DOUBLE_EQ(new_i_gain, hc1_tmp.getPIDGains().i_gain_);
  EXPECT_DOUBLE_EQ(new_d_gain, hc1_tmp.getPIDGains().d_gain_);

  ActuatorHandle hc2_tmp = iface.getHandle(name2);
  EXPECT_EQ(name2, hc2_tmp.getName());
  EXPECT_DOUBLE_EQ(pos2, hc2_tmp.getPosition());
  EXPECT_DOUBLE_EQ(vel2, hc2_tmp.getVelocity());
  EXPECT_DOUBLE_EQ(eff2, hc2_tmp.getEffort());
  EXPECT_DOUBLE_EQ(cmd2, hc2_tmp.getCommand());
  const double new_cmd_2 = -2.0;
  hc2_tmp.setCommand(new_cmd_2);
  EXPECT_DOUBLE_EQ(new_cmd_2, hc2_tmp.getCommand());

  // This interface does not claim resources
  EXPECT_TRUE(iface.getClaims().empty());

  // Print error message
  // Requires manual output inspection, but exception message should contain the interface name (not its base class)
  try {iface.getHandle("unknown_name");}
  catch(const HardwareInterfaceException& e) {ROS_ERROR_STREAM(e.what());}
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

