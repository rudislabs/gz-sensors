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

#if defined(_MSC_VER)
  #pragma warning(push)
  #pragma warning(disable: 4005)
  #pragma warning(disable: 4251)
#endif
#include <gz/msgs/uwb.pb.h>
#if defined(_MSC_VER)
  #pragma warning(pop)
#endif

#include <gz/common/Profiler.hh>
#include <gz/msgs/Utility.hh>
#include <gz/transport/Node.hh>

#include "gz/sensors/UwbSensor.hh"
#include "gz/sensors/SensorFactory.hh"
#include "gz/sensors/SensorTypes.hh"

using namespace gz;
using namespace sensors;

/// \brief Private data for UwbSensor
class gz::sensors::UwbSensorPrivate
{
  /// \brief node to create publisher
  public: transport::Node node;

  /// \brief publisher to publish uwb messages.
  public: transport::Node::Publisher pub;

  /// \brief true if Load() has been called and was successful
  public: bool initialized = false;
 
  /// \brief Previous update time step.
  public: std::chrono::steady_clock::duration prevStep
    {std::chrono::steady_clock::duration::zero()};

  /// \brief Flag for if time has been initialized
  public: bool timeInitialized = false;

  /// \brief transform to Imu orientation reference frame.
  public: math::Quaterniond orientationReference;

  /// \brief transform to Imu frame from Imu reference frame.
  public: math::Quaterniond orientation;

  /// \brief World pose of the imu sensor
  public: math::Pose3d worldPose;

  /// \brief Orientation of world frame relative to a specified frame
  public: math::Quaterniond worldRelativeOrientation;

  /// \brief Frame relative-to which the worldRelativeOrientation
  //  is defined
  public: WorldFrameEnumType worldFrameRelativeTo = WorldFrameEnumType::NED;

  /// \brief Frame relative-to which the sensor orientation is reported
  public: WorldFrameEnumType sensorOrientationRelativeTo
         = WorldFrameEnumType::NONE;

  /// \brief The azimuth of the UWB sensor in relation to the partner UWB sensor
  /// public: float azimuth;
};

//////////////////////////////////////////////////
UwbSensor::UwbSensor()
  : dataPtr(new UwbSensorPrivate())
{
}

//////////////////////////////////////////////////
UwbSensor::~UwbSensor()
{
}

//////////////////////////////////////////////////
bool UwbSensor::Init()
{
  return this->Sensor::Init();
}

//////////////////////////////////////////////////
bool UwbSensor::Load(const sdf::Sensor &_sdf)
{
  if (!Sensor::Load(_sdf))
    return false;

  // Check if this is the right type
  if (_sdf.Type() != sdf::SensorType::UWB)
  {
    gzerr << "Attempting to a load an UWB sensor, but received "
      << "a " << _sdf.TypeStr() << std::endl;
  }

  // Register publisher
  if (_sdf.UwbSensor() == nullptr)
  {
    gzerr << "Attempting to a load an UWB sensor, but received "
      << "a null sensor." << std::endl;
    return false;
  }

  if (this->Topic().empty())
    this->SetTopic("/uwb");

  this->dataPtr->pub =
      this->dataPtr->node.Advertise<msgs::UWB>(this->Topic());

  if (!this->dataPtr->pub)
  {
    gzerr << "Unable to create publisher on topic[" << this->Topic() << "].\n";
    return false;
  }

  gzdbg << "UWB data for [" << this->Name() << "] advertised on ["
         << this->Topic() << "]" << std::endl;

  std::string localization = _sdf.UwbSensor()->Localization();

  if (localization == "ENU")
  {
    this->dataPtr->sensorOrientationRelativeTo = WorldFrameEnumType::ENU;
  } else if (localization == "NED") {
    this->dataPtr->sensorOrientationRelativeTo = WorldFrameEnumType::NED;
  } else if (localization == "NWU") {
    this->dataPtr->sensorOrientationRelativeTo = WorldFrameEnumType::NWU;
  } else {
    this->dataPtr->sensorOrientationRelativeTo = WorldFrameEnumType::NONE;
  }

  this->dataPtr->initialized = true;
  return true;
}

//////////////////////////////////////////////////
bool UwbSensor::Load(sdf::ElementPtr _sdf)
{
  sdf::Sensor sdfSensor;
  sdfSensor.Load(_sdf);
  return this->Load(sdfSensor);
}

//////////////////////////////////////////////////
bool UwbSensor::Update(const std::chrono::steady_clock::duration &_now)
{
  GZ_PROFILE("UwbSensor::Update");
  if (!this->dataPtr->initialized)
  {
    gzerr << "Not initialized, update ignored.\n";
    return false;
  }

  // If time has gone backwards, reinitialize.
  if (_now < this->dataPtr->prevStep)
  {
    this->dataPtr->timeInitialized = false;
  }

  msgs::UWB msg;
  *msg.mutable_header()->mutable_stamp() = msgs::Convert(_now);
  msg.set_entity_name(this->Name());
  auto frame = msg.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(this->FrameId());

  // Set the time stamp
/*   *this->dataPtr->pointMsg.mutable_header()->mutable_stamp() =
    msgs::Convert(_now); */

    // Set the UWB orientation
    // uwb orientation with respect to reference frame
  this->dataPtr->orientation =
      this->dataPtr->orientationReference.Inverse() *
      this->dataPtr->worldPose.Rot();

  msgs::Set(msg.mutable_orientation(), this->dataPtr->orientation);

  // Calculate azimuth angle of device in relation to partner device
  // TODO: Incorporate orientation
  // working under the assumption that the controllee UWB sensor is always pointing North
  /// azimuth = atan(this->Pose().Pos().X()/ this->Pose().Pos().Y());

  /// msg.set_aoa_azimuth_dev(azimuth);
  
  // publish
  this->AddSequence(msg.mutable_header());
  this->dataPtr->pub.Publish(msg);
  this->dataPtr->prevStep = _now;
  this->dataPtr->timeInitialized = true;
  return true;
}

//////////////////////////////////////////////////
void UwbSensor::SetWorldPose(const math::Pose3d _pose)
{
  this->dataPtr->worldPose = _pose;
}

//////////////////////////////////////////////////
math::Pose3d UwbSensor::WorldPose() const
{
  return this->dataPtr->worldPose;
}


//////////////////////////////////////////////////
void UwbSensor::SetWorldFrameOrientation(
  const math::Quaterniond &_rot, WorldFrameEnumType _relativeTo)
{
  this->dataPtr->worldRelativeOrientation = _rot;
  this->dataPtr->worldFrameRelativeTo = _relativeTo;

  // Table to hold frame transformations
  static const std::map<WorldFrameEnumType,
    std::map<WorldFrameEnumType, math::Quaterniond>>
      transformTable =
    {
      {WorldFrameEnumType::ENU,
        {
          {WorldFrameEnumType::ENU, math::Quaterniond(0, 0, 0)},
          {WorldFrameEnumType::NED, math::Quaterniond(
            GZ_PI, 0, GZ_PI/2)},
          {WorldFrameEnumType::NWU, math::Quaterniond(
            0, 0, GZ_PI/2)},
        }
      },
      {WorldFrameEnumType::NED,
        {
          {WorldFrameEnumType::ENU, math::Quaterniond(
            GZ_PI, 0, GZ_PI/2).Inverse()},
          {WorldFrameEnumType::NED, math::Quaterniond(0, 0, 0)},
          {WorldFrameEnumType::NWU, math::Quaterniond(
            -GZ_PI, 0, 0)},
        }
      },
      {WorldFrameEnumType::NWU,
        {
          {WorldFrameEnumType::ENU, math::Quaterniond(
            0, 0, -GZ_PI/2)},
          {WorldFrameEnumType::NED, math::Quaterniond(GZ_PI, 0, 0)},
          {WorldFrameEnumType::NWU, math::Quaterniond(0, 0, 0)},
        }
      }
    };
}

//////////////////////////////////////////////////
math::Quaterniond UwbSensor::OrientationReference() const
{
  return this->dataPtr->orientationReference;
}

//////////////////////////////////////////////////
math::Quaterniond UwbSensor::Orientation() const
{
  return this->dataPtr->orientation;
}

//////////////////////////////////////////////////
bool UwbSensor::HasConnections() const
{
  return this->dataPtr->pub && this->dataPtr->pub.HasConnections();
}
