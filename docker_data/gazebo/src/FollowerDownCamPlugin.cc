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

#include "FollowerDownCamPlugin.hh"

#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/World.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <vector>

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(FollowerDownCamPlugin)

/////////////////////////////////////////////////
void FollowerDownCamPlugin::Load(physics::ModelPtr _model,
                                 sdf::ElementPtr _sdf)
{
  if (!_model)
  {
    gzerr << "[FollowerDownCamPlugin] Invalid model pointer.\n";
    return;
  }

  this->model = _model;

  if (_sdf)
  {
    if (_sdf->HasElement("target_model"))
    {
      this->targetModelName = _sdf->Get<std::string>("target_model");
    }
    if (_sdf->HasElement("target_link"))
    {
      this->targetLinkName = _sdf->Get<std::string>("target_link");
    }
    if (_sdf->HasElement("offset"))
    {
      this->offset = _sdf->Get<ignition::math::Vector3d>("offset");
    }
  }

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&FollowerDownCamPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void FollowerDownCamPlugin::ResolveTarget()
{
  if (!this->model)
    return;

  auto world = this->model->GetWorld();
  if (!world)
    return;

  if (!this->targetModel)
  {
    this->targetModel = world->ModelByName(this->targetModelName);
    if (!this->targetModel && !this->warnedMissingTarget)
    {
      gzwarn << "[FollowerDownCamPlugin] Target model [" << this->targetModelName
             << "] was not found yet. Waiting for it to spawn...\n";
      this->warnedMissingTarget = true;
    }
  }

  if (this->targetModel && !this->targetLink)
  {
    std::vector<std::string> candidates;
    candidates.push_back(this->targetLinkName);
    if (this->targetLinkName.find("::") == std::string::npos)
    {
      candidates.push_back(this->targetModel->GetScopedName() + "::" +
                           this->targetLinkName);
      candidates.push_back(this->targetModel->GetName() + "::" +
                           this->targetLinkName);
    }

    for (const auto &linkName : candidates)
    {
      if (linkName.empty())
        continue;

      this->targetLink = this->targetModel->GetLink(linkName);
      if (this->targetLink)
        break;
    }

    if (!this->targetLink && !this->warnedMissingLink)
    {
      gzwarn << "[FollowerDownCamPlugin] Unable to find link [" << this->targetLinkName
             << "] inside model [" << this->targetModelName << "].\n";
      this->warnedMissingLink = true;
    }
  }
}

/////////////////////////////////////////////////
void FollowerDownCamPlugin::OnUpdate()
{
  if (!this->targetLink)
  {
    this->ResolveTarget();
  }

  if (!this->targetLink)
    return;

  ignition::math::Pose3d targetPose = this->targetLink->WorldPose();
  ignition::math::Vector3d desiredPos =
      targetPose.Pos() + targetPose.Rot() * this->offset;

  // Keep the camera level with the world frame so the optical axis always
  // matches -Z irrespective of the target attitude.
  static const ignition::math::Quaterniond nadirOrientation(0, 0, 0);
  ignition::math::Pose3d desiredPose(desiredPos, nadirOrientation);

  this->model->SetWorldPose(desiredPose);
}
