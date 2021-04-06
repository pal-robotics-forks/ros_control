///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2012, hiDOF INC.
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

#ifndef HARDWARE_INTERFACE_ACTUATOR_COMMAND_INTERFACE_H
#define HARDWARE_INTERFACE_ACTUATOR_COMMAND_INTERFACE_H

#include <string>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/actuator_state_interface.h>
#include <control_toolbox/pid.h>

namespace hardware_interface
{

/** \brief A handle used to read and command a single actuator. */
class ActuatorHandle : public ActuatorStateHandle
{
public:
  ActuatorHandle() : ActuatorStateHandle(), cmd_(0)
  {
    initializeGains();
  }

  /**
   * \param as This actuator's state handle
   * \param cmd A pointer to the storage for this actuator's output command
   */
  ActuatorHandle(const ActuatorStateHandle& as, double* cmd)
    : ActuatorStateHandle(as), cmd_(cmd)
  {
    if (!cmd_)
    {
      throw HardwareInterfaceException("Cannot create handle '" + as.getName() +
                                       "'. Command data pointer is null.");
    }
    initializeGains();
  }

  void setCommand(double command) {assert(cmd_); *cmd_ = command;}
  double getCommand() const {assert(cmd_); return *cmd_;}

  double* getCommandPtr() {return cmd_;}

  // Methods for setting and getting the gains information

  void setPIDGains(double p, double i, double d)
  {
    pid_gains_.p_gain_ = p;
    pid_gains_.i_gain_ = i;
    pid_gains_.d_gain_ = d;
  }

  control_toolbox::Pid::Gains getPIDGains() const
  {
    return pid_gains_;
  }

  const control_toolbox::Pid::Gains* getPIDGainsConstPtr() const
  {
    return &pid_gains_;
  }

  void setFFGain(double ff_gain)
  {
    ff_gain_ = ff_gain;
  }

  double getFFGain() const
  {
    return ff_gain_;
  }

  const double* getFFGainConstPtr() const
  {
    return &ff_gain_;
  }

private:
  void initializeGains()
  {
    const double nan_value = std::numeric_limits<double>::quiet_NaN();
    pid_gains_ =
        control_toolbox::Pid::Gains(nan_value, nan_value, nan_value, nan_value, nan_value);
    ff_gain_ = nan_value;
  }

  double* cmd_;
  control_toolbox::Pid::Gains pid_gains_;
  double ff_gain_;
};

/** \brief Hardware interface to support commanding an array of actuators.
 *
 * This \ref HardwareInterface supports commanding the output of an array of
 * named actuators. Note that these commands can have any semantic meaning as long
 * as they each can be represented by a single double, they are not necessarily
 * effort commands. To specify a meaning to this command, see the derived
 * classes like \ref EffortActuatorInterface etc.
 *
 */
class ActuatorCommandInterface : public HardwareResourceManager<ActuatorHandle> {};

/// \ref ActuatorCommandInterface for commanding effort-based actuators
class EffortActuatorInterface : public ActuatorCommandInterface {};

/// \ref ActuatorCommandInterface for commanding velocity-based actuators
class VelocityActuatorInterface : public ActuatorCommandInterface {};

/// \ref ActuatorCommandInterface for commanding position-based actuators
class PositionActuatorInterface : public ActuatorCommandInterface {};

}

#endif
