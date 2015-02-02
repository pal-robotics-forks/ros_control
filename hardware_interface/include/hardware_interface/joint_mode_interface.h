/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, University of Colorado, Boulder
 *  Copyright (c) 2014, PAL Robotics SL.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef HARDWARE_INTERFACE_JOINT_MODE_INTERFACE_H
#define HARDWARE_INTERFACE_JOINT_MODE_INTERFACE_H

#include <cassert>
#include <hardware_interface/internal/hardware_resource_manager.h>

namespace
{
  // typedef local to this file
  typedef std::set<std::string> StringSet;
}

namespace hardware_interface
{

/**
 * \brief A handle used to read and write the mode of a single joint.
 * \author: Bence Magyar, Dave Coleman
 */
class JointModeHandle
{
public:

  /**
   * \param name Name of the joint
   * \param mode Pointer to the current joint mode
   * \param start_mode The joint mode to start in (if there is any)
   * \param allowed_modes A set of hardware interfaces names. If not given, setMode() will allow any joint mode to be set. If given, setMode() will refuse and return false when setting to a joint mode that is not in this set.
   */
  JointModeHandle(const std::string& name, std::string* mode,
                  const std::string& start_mode = "",
                  const StringSet& allowed_modes = StringSet())
    : mode_(mode)
    , name_(name)
  {
    if (!mode_)
    {
      throw HardwareInterfaceException("Cannot create handle '" + name + "'. Mode data pointer is null.");
    }
    allowed_modes_.reset(new StringSet(allowed_modes));
    *mode = start_mode;
  }

  const std::string& getName() const {return name_;}

  /**
   * \brief Sets the mode of the joint. If there are 0 allowed modes registered
   * any mode change will pass. If there are modes registered, only the registered
   * ones will pass.
   */
  bool setMode(const std::string& mode)
  {
    if(allowed_modes_->empty() || allowed_modes_->count(mode) > 0)
    {
      *mode_ = mode;
      return true;
    }
    else
    {
      return false;
    }
  }

  const std::string& getMode() const
  {
    assert(mode_);
    return *mode_;
  }

  boost::shared_ptr<StringSet> getAvailableModes() const 
  { return allowed_modes_; }

private:
  std::string* mode_;
  std::string name_;
  boost::shared_ptr<StringSet> allowed_modes_;
};

/**
 *\brief Hardware interface to support changing between control modes
 */
class JointModeInterface : public HardwareResourceManager<JointModeHandle, ClaimResources>{};

} // namespace

#endif
