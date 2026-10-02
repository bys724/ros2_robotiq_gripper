// Copyright (c) 2022 PickNik, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <cstdint>

/**
 * @brief This interface describes how to communicate with the gripper hardware.
 */
namespace robotiq_driver
{
class Driver
{
public:
  enum class ActivationStatus
  {
    RESET,
    ACTIVE
  };

  enum class ActionStatus
  {
    STOPPED,
    MOVING
  };

  enum class GripperStatus
  {
    RESET,
    IN_PROGRESS,
    COMPLETED,
  };

  enum class ObjectDetectionStatus
  {
    MOVING,
    OBJECT_DETECTED_OPENING,
    OBJECT_DETECTED_CLOSING,
    AT_REQUESTED_POSITION
  };

  virtual void set_slave_address(uint8_t slave_address) = 0;

  /** Connect to the gripper serial connection. */
  virtual bool connect() = 0;

  /** Disconnect from the gripper serial connection. */
  virtual void disconnect() = 0;

  /**
   * @brief Activates the gripper.
   * @throw serial::IOException on failure to successfully communicate with gripper port
   */
  virtual void activate() = 0;

  /**
   * @brief Deactivates the gripper.
   * @throw serial::IOException on failure to successfully communicate with gripper port
   */
  virtual void deactivate() = 0;

  /**
   * @brief Commands the gripper to move to the desired position.
   * @param pos A value between 0x00 (fully open) and 0xFF (fully closed).
   */
  virtual void set_gripper_position(uint8_t pos) = 0;

  /**
   * @brief Return the current position of the gripper.
   *
   * @throw serial::IOException on failure to successfully communicate with gripper port
   *
   * @return uint8_t A value between 0x00 (fully open) and 0xFF (fully closed).
   */
  virtual uint8_t get_gripper_position() = 0;

  /**
   * @brief Returns true if the gripper is currently moving, false otherwise.
   */
  virtual bool gripper_is_moving() = 0;

  /**
   * @brief Set the speed of the gripper.
   *
   * @param speed A value between 0x00 (stopped) and 0xFF (full speed).
   */
  virtual void set_speed(uint8_t speed) = 0;

  /**
   * @brief Set how forcefully the gripper opens or closes.
   * @param force A value between 0x00 (no force) or 0xFF (maximum force).
   */
  virtual void set_force(uint8_t force) = 0;

  // --- Status getters for the `gripper_status` GPIO state interfaces (re-creation of lost patch 8f6c893) ---
  // They return the values cached by the last status read (get_gripper_position / gripper_is_moving),
  // so they do not add Modbus round trips. Defaults keep fake/mock drivers source-compatible.

  /** @brief Last object detection status (gOBJ). */
  virtual ObjectDetectionStatus get_object_detection_status()
  {
    return ObjectDetectionStatus::MOVING;
  }

  /** @brief Last activation status (gACT). */
  virtual ActivationStatus get_activation_status()
  {
    return ActivationStatus::RESET;
  }

  /** @brief Last fault status register (gFLT byte, 0x00 = no fault). */
  virtual uint8_t get_fault_status()
  {
    return 0;
  }

  /** @brief Speed currently commanded to the gripper (rSP, 0x00-0xFF). */
  virtual uint8_t get_commanded_speed()
  {
    return 0;
  }

  /** @brief Force currently commanded to the gripper (rFR, 0x00-0xFF). */
  virtual uint8_t get_commanded_force()
  {
    return 0;
  }
};
}  // namespace robotiq_driver
