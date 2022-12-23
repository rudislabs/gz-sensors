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
#include <gz/msgs/fluid_pressure.pb.h>
#if defined(_MSC_VER)
  #pragma warning(pop)
#endif
#include <gz/msgs/Utility.hh>

#include <gz/common/Profiler.hh>
#include <gz/transport/Node.hh>

#include "gz/sensors/GaussianNoiseModel.hh"
#include "gz/sensors/Noise.hh"
#include "gz/sensors/SensorTypes.hh"
#include "gz/sensors/SensorFactory.hh"
#include "gz/sensors/AirSpeedSensor.hh"

using namespace gz;
using namespace sensors;

static constexpr double kPressureOneAtmospherePascals = 101325.0;
// static constexpr double kSeaLevelTempKelvin = 288.15;

static constexpr auto DEFAULT_HOME_ALT_AMSL = 488.0f; // altitude AMSL at Irchel Park, Zurich, Switzerland [m]

// international standard atmosphere (troposphere model - valid up to 11km) see [1]
static constexpr auto TEMPERATURE_MSL = 288.15f; // temperature at MSL [K] (15 [C])
static constexpr auto PRESSURE_MSL = 101325.0f; // pressure at MSL [Pa]
static constexpr auto LAPSE_RATE = 0.0065f; // reduction in temperature with altitude for troposphere [K/m]
static constexpr auto AIR_DENSITY_MSL = 1.225f; // air density at MSL [kg/m^3]

/// \brief Private data for AirSpeedSensor
class gz::sensors::AirSpeedSensorPrivate
{
  /// \brief node to create publisher
  public: transport::Node node;

  /// \brief publisher to publish air pressure messages.
  public: transport::Node::Publisher pub;

  /// \brief true if Load() has been called and was successful
  public: bool initialized = false;

  /// \brief Pressure in pascals.
  public: double pressure = 0.0;

  /// \brief Pose of the sensor
  public: gz::math::Vector3d vel;

  /// \brief Noise added to sensor data
  public: std::map<SensorNoiseType, NoisePtr> noises;
};

//////////////////////////////////////////////////
AirSpeedSensor::AirSpeedSensor()
  : dataPtr(new AirSpeedSensorPrivate())
{
}

//////////////////////////////////////////////////
AirSpeedSensor::~AirSpeedSensor()
{
}

//////////////////////////////////////////////////
bool AirSpeedSensor::Init()
{
  return this->Sensor::Init();
}

//////////////////////////////////////////////////
bool AirSpeedSensor::Load(const sdf::Sensor &_sdf)
{
  if (!Sensor::Load(_sdf))
    return false;

  if (_sdf.Type() != sdf::SensorType::AIR_SPEED)
  {
    gzerr << "Attempting to a load an AirSpeed sensor, but received "
      << "a " << _sdf.TypeStr() << std::endl;
    return false;
  }

  if (_sdf.AirSpeedSensor() == nullptr)
  {
    gzerr << "Attempting to a load an AirSpeed sensor, but received "
      << "a null sensor." << std::endl;
    return false;
  }

  if (this->Topic().empty())
    this->SetTopic("/air_speed");

  this->dataPtr->pub =
      this->dataPtr->node.Advertise<msgs::FluidPressure>(
      this->Topic());

  if (!this->dataPtr->pub)
  {
    gzerr << "Unable to create publisher on topic[" << this->Topic() << "].\n";
    return false;
  }

  gzdbg << "Air speed for [" << this->Name() << "] advertised on ["
         << this->Topic() << "]" << std::endl;

  // Load the noise parameters
  if (_sdf.AirSpeedSensor()->SpeedNoise().Type() != sdf::NoiseType::NONE)
  {
    this->dataPtr->noises[AIR_SPEED_NOISE_PASCALS] =
      NoiseFactory::NewNoiseModel(_sdf.AirSpeedSensor()->SpeedNoise());
  }

  this->dataPtr->initialized = true;
  return true;
}

//////////////////////////////////////////////////
bool AirSpeedSensor::Load(sdf::ElementPtr _sdf)
{
  sdf::Sensor sdfSensor;
  sdfSensor.Load(_sdf);
  return this->Load(sdfSensor);
}

//////////////////////////////////////////////////
bool AirSpeedSensor::Update(
  const std::chrono::steady_clock::duration &_now)
{
  GZ_PROFILE("AirSpeedSensor::Update");
  if (!this->dataPtr->initialized)
  {
    gzerr << "Not initialized, update ignored.\n";
    return false;
  }

  msgs::FluidPressure msg;
  *msg.mutable_header()->mutable_stamp() = msgs::Convert(_now);
  auto frame = msg.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(this->FrameId());

  // // This block of code comes from RotorS:
  // // https://github.com/ethz-asl/rotors_simulator/blob/master/rotors_gazebo_plugins/src/gazebo_pressure_plugin.cpp
  // {
  //   // Get the current height.
  //   double height = this->dataPtr->referenceAltitude + this->Pose().Pos().Z();
  //
  //   // Compute the geopotential height.
  //   double geoHeight = kEarthRadiusMeters * height /
  //     (kEarthRadiusMeters + height);
  //
  //   // Compute the temperature at the current altitude in Kelvin.
  //   double tempAtHeight =
  //     kSeaLevelTempKelvin - kTempLapseKelvinPerMeter * geoHeight;
  //
  //   // Compute the current air pressure.
  //   this->dataPtr->pressure =
  //     kPressureOneAtmospherePascals * exp(kAirConstantDimensionless *
  //         log(kSeaLevelTempKelvin / tempAtHeight));
  // }

  const float alt_rel = static_cast<float>(this->Pose().Pos().Z()); // Z-component from ENU
  const float alt_amsl = (float)DEFAULT_HOME_ALT_AMSL + alt_rel;
  const float temperature_local = TEMPERATURE_MSL - LAPSE_RATE * alt_amsl;
  const float density_ratio = powf(TEMPERATURE_MSL / temperature_local , 4.256f);
  const float air_density = AIR_DENSITY_MSL / density_ratio;
	std::cerr << "air_density " << air_density << '\n';
  // 	// calculate differential pressure + noise in hPa

	gz::math::Vector3d wind_vel_{0, 0, 0};
  gz::math::Quaterniond veh_q_world_to_body = this->Pose().Rot();
	gz::math::Vector3d air_vel_in_body_ = this->dataPtr->vel - veh_q_world_to_body.RotateVectorReverse(wind_vel_);
	double diff_pressure = gz::math::sgn(air_vel_in_body_.X()) * 0.005f * air_density  * air_vel_in_body_.X() * air_vel_in_body_.X();
  std::cerr << "diff_pressure " << diff_pressure << '\n';
  std::cerr << "this->dataPtr->vel " << this->dataPtr->vel << '\n';
  std::cerr << "veh_q_world_to_body " << veh_q_world_to_body << '\n';

  // Apply pressure noise
  if (this->dataPtr->noises.find(AIR_SPEED_NOISE_PASCALS) !=
      this->dataPtr->noises.end())
  {
    diff_pressure =
      this->dataPtr->noises[AIR_SPEED_NOISE_PASCALS]->Apply(
          diff_pressure);

    if (this->dataPtr->noises[AIR_SPEED_NOISE_PASCALS]->Type() ==
        NoiseType::GAUSSIAN)
    {
     GaussianNoiseModelPtr gaussian =
       std::dynamic_pointer_cast<GaussianNoiseModel>(
           this->dataPtr->noises[AIR_SPEED_NOISE_PASCALS]);
      msg.set_variance(sqrt(gaussian->StdDev()));
    }
  }

  msg.set_pressure(diff_pressure);
  msg.set_variance(temperature_local);

  // publish
  this->AddSequence(msg.mutable_header());
  this->dataPtr->pub.Publish(msg);

  return true;
}

//////////////////////////////////////////////////
gz::math::Vector3d AirSpeedSensor::Velocity() const
{
  return this->dataPtr->vel;
}

//////////////////////////////////////////////////
void AirSpeedSensor::SetVelocity(const gz::math::Vector3d &_vel)
{
  this->dataPtr->vel = _vel;
}

//////////////////////////////////////////////////
bool AirSpeedSensor::HasConnections() const
{
  return this->dataPtr->pub && this->dataPtr->pub.HasConnections();
}
