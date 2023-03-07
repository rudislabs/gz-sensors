/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifndef GZ_SENSORS_UWBSENSOR_HH_
#define GZ_SENSORS_UWBSENSOR_HH_

#include <memory>

#include <sdf/sdf.hh>

#include <gz/utils/SuppressWarning.hh>
#include <gz/math/Pose3.hh>

#include <gz/sensors/config.hh>
#include <gz/sensors/uwb/Export.hh>

#include "gz/sensors/Sensor.hh"

namespace gz
{
  namespace sensors
  {
    /// Inline bracket to help doxygen filtering.
    inline namespace GZ_SENSORS_VERSION_NAMESPACE {

/*     /// \brief Reference frames enum
    enum class GZ_SENSORS_VISIBLE WorldFrameEnumType
    {
      /// \brief NONE : To be used only when <localization>
      /// reference orientation tag is empty.
      NONE = 0,

      /// \brief ENU (East-North-Up): x - East, y - North, z - Up.
      ENU = 1,

      /// \brief NED (North-East-Down): x - North, y - East, z - Down.
      NED = 2,

      /// \brief NWU (North-West-Up): x - North, y - West, z - Up.
      NWU = 3,
    }; */

    ///
    /// \brief forward declarations
    class UwbSensorPrivate;

    /// \brief Uwb Sensor Class
    ///
    /// An uwb sensor that reports linear acceleration, angular velocity, and
    /// orientation
    class GZ_SENSORS_UWB_VISIBLE UwbSensor : public Sensor
    {
      /// \brief constructor
      public: UwbSensor();

      /// \brief destructor
      public: virtual ~UwbSensor();

      /// \brief Load the sensor based on data from an sdf::Sensor object.
      /// \param[in] _sdf SDF Sensor parameters.
      /// \return true if loading was successful
      public: virtual bool Load(const sdf::Sensor &_sdf) override;

      /// \brief Load the sensor with SDF parameters.
      /// \param[in] _sdf SDF Sensor parameters.
      /// \return true if loading was successful
      public: virtual bool Load(sdf::ElementPtr _sdf) override;

      /// \brief Initialize values in the sensor
      /// \return True on success
      public: virtual bool Init() override;

      using Sensor::Update;

      /// \brief Update the sensor and generate data
      /// \param[in] _now The current time
      /// \return true if the update was successful
      public: virtual bool Update(
        const std::chrono::steady_clock::duration &_now) override;

      /// \brief Set the world pose of the uwb sensor
      /// \param[in] _pose Pose in world frame
      public: void SetWorldPose(const math::Pose3d _pose);

      /// \brief Get the world pose of the uwb sensor
      /// \return Pose in world frame.
      public: math::Pose3d WorldPose() const;

/*       /// \brief Get the Azimuth of the controller relative to NED of controllee UWB sensor
      /// \return Azimuth in degrees
      public: float azimuth_dev(const math::Pose3d _pose) const;

      /// \brief Get the world pose of the uwb sensor
      /// \return Pose in world frame.
      public: void SetAzimuth(float &_azimuth_dev); */

      /// TODO: Need to access the pose of the other uwb device. but how?
      /// \brief Get the Elevation of the controller relative to NED of controllee UWB sensor
      /// \return Elevation in degrees
      ///public: float elevation_dev(const math::Pose3d _pose);

      /// \brief Check if there are any subscribers for sensor data
      /// \return True if there are subscribers, false otherwise
      public: virtual bool HasConnections() const override;

      GZ_UTILS_WARN_IGNORE__DLL_INTERFACE_MISSING
      /// \brief Data pointer for private data
      /// \internal
      private: std::unique_ptr<UwbSensorPrivate> dataPtr;
      GZ_UTILS_WARN_RESUME__DLL_INTERFACE_MISSING
    };
    }
  }
}

#endif
