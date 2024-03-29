﻿/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 	* Redistributions of source code must retain the above copyright
 * 	notice, this list of conditions and the following disclaimer.
 * 	* Redistributions in binary form must reproduce the above copyright
 * 	notice, this list of conditions and the following disclaimer in the
 * 	documentation and/or other materials provided with the distribution.
 * 	* Neither the name of the Southwest Research Institute, nor the names
 *	of its contributors may be used to endorse or promote products derived
 *	from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef JOINT_MESSAGE_H
#define JOINT_MESSAGE_H

#ifdef ROS
#include "simple_message/typed_message.h"
#include "simple_message/simple_message.h"
#include "simple_message/shared_types.h"
#include "simple_message/joint_data.h"
#endif

#ifdef MOTOPLUS
#include "typed_message.h"
#include "simple_message.h"
#include "shared_types.h"
#include "joint_data.h"
#endif

namespace industrial
{
namespace joint_message
{

// TODO: The JointMessage is currently overloaded (i.e. a single message means multiple
// things).  This is not ideal and seperate messages should be considered.
/**
 * \brief Enumeration of special sequence values that signal the end of trajectory
 * or an immediate stop.
 */
namespace SpecialSeqValues
{
enum SpecialSeqValue
{
  END_TRAJECTORY = -1, STOP_TRAJECTORY = -2
};
}
typedef SpecialSeqValues::SpecialSeqValue SpecialSeqValue;



/**
 * \brief Class encapsulated joint message generation methods (either to or
 * from a SimpleMessage type.
 */
//* JointMessage
/**
 * The JOINT message structure is meant to contain joint position information
 * either related to a trajectory point or a joint feedback message.  The
 * data structure is as follow:
 *
 * int SEQ_NUM identifies the order within a trajectory sequence (not valid for feedback)
 * real JOINTS[MAX_NUM_JOINTS] joint values.  The number of joints are fixed and assumed
 *                              to be in a known order (defined both by the order in ROS
 *                              and the order as defined by the robot.
 *
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class JointMessage : public industrial::typed_message::TypedMessage
{
public:
  /**
   * \brief Default constructor
   *
   * This method creates an empty message.
   *
   */
  JointMessage(void);
  /**
   * \brief Destructor
   *
   */
  ~JointMessage(void);
  /**
   * \brief Initializes message from a simple message
   *
   * \param simple message to construct from
   *
   * \return true if message successfully initialized, otherwise false
   */
  bool init(industrial::simple_message::SimpleMessage & msg);

  /**
   * \brief Initializes message from a joint structure
   *
   * \param sequence number
   * \param joints
   *
   */
  void init(industrial::shared_types::shared_int seq, industrial::joint_data::JointData & joints);

  /**
   * \brief Initializes a new joint message
   *
   */
  void init();

  /**
   * \brief Sets message sequence number
   *
   * \param message sequence number
   */
  void setSequence(industrial::shared_types::shared_int sequence);

  /**
   * \brief returns the maximum message sequence number
   *
   * \return message sequence number
   */
  industrial::shared_types::shared_int getSequence()
  {
    return sequence_;
  }

  /**
   * \brief returns reference to underlying joint class
   *
   * \return reference to joint class
   */
  industrial::joint_data::JointData& getJoints()
  {
    return this->joints_;
  }

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);


  unsigned int byteLength()
  {
    return sizeof(industrial::shared_types::shared_int) + this->joints_.byteLength();
  }

private:
  /**
   * \brief sequence number (for those joints messages that require it)
   */
  industrial::shared_types::shared_int sequence_;
  /**
   * \brief maximum number of joints positions that can be held in the message.
   */
  industrial::joint_data::JointData joints_;

};

}
}

#endif /* JOINT_MESSAGE_H */
