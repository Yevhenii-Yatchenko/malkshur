/*
 * Copyright (C) 2024
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
#ifndef GAZEBO_PLUGINS_FOLLOWERDOWNCAMPLUGIN_HH_
#define GAZEBO_PLUGINS_FOLLOWERDOWNCAMPLUGIN_HH_

#include <string>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/util/system.hh"
#include "ignition/math/Vector3.hh"

namespace gazebo
{
  /// \brief Simple plugin that repositions and re-orients a camera model so it
  /// always looks straight down at a target vehicle.
  class GAZEBO_VISIBLE FollowerDownCamPlugin : public ModelPlugin
  {
    /// \brief Load configuration from SDF.
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

    /// \brief Update callback.
    private: void OnUpdate();

    /// \brief Try to resolve pointers to the target model/link.
    private: void ResolveTarget();

    /// \brief Owning camera model.
    private: physics::ModelPtr model;

    /// \brief Model we are following.
    private: physics::ModelPtr targetModel;

    /// \brief Link we are following.
    private: physics::LinkPtr targetLink;

    /// \brief Offset applied in the target-link frame.
    private: ignition::math::Vector3d offset{0, 0, -1};

    /// \brief Name of the target model.
    private: std::string targetModelName = "iris_demo";

    /// \brief Name of the target link (can be scoped).
    private: std::string targetLinkName = "iris::base_link";

    /// \brief Connection to the world update event.
    private: event::ConnectionPtr updateConnection;

    /// \brief Track if warnings were printed to avoid spamming.
    private: bool warnedMissingTarget = false;
    private: bool warnedMissingLink = false;
  };
}
#endif
