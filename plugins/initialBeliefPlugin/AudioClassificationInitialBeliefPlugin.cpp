/**
 * Copyright 2017
 *
 * This file is part of On-line POMDP Planning Toolkit (OPPT).
 * OPPT is free software: you can redistribute it and/or modify it under the terms of the
 * GNU General Public License published by the Free Software Foundation,
 * either version 2 of the License, or (at your option) any later version.
 *
 * OPPT is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with OPPT.
 * If not, see http://www.gnu.org/licenses/.
 */
#ifndef _DEFAULT_INITIAL_STATE_SAMPLER_PLUGIN_HPP_
#define _DEFAULT_INITIAL_STATE_SAMPLER_PLUGIN_HPP_
#include <oppt/plugin/Plugin.hpp>
#include <oppt/gazeboInterface/GazeboInterface.hpp>
#include "AudioClassificationInitialBeliefOptions.hpp"
#include "AudioClassificationUserData.hpp"

namespace oppt
{
class AudioClassificationInitialBeliefPlugin: public InitialBeliefPlugin
{
public:
    AudioClassificationInitialBeliefPlugin():
        InitialBeliefPlugin() {
    }

    virtual ~AudioClassificationInitialBeliefPlugin() = default;

    virtual bool load(RobotEnvironment* const robotEnvironment,
                      const std::string& optionsFile) override {
        robotEnvironment_ = robotEnvironment;
        parseOptions_<AudioClassificationInitialBeliefOptions>(optionsFile);
        initialStateVector_ = static_cast<const AudioClassificationInitialBeliefOptions *>(options_.get())->initialState;

        // Get a pointer to the end-effector link
        std::string endEffectorLinkName =
            static_cast<const AudioClassificationInitialBeliefOptions *>(options_.get())->endEffectorLink;
        endEffectorLink_ = getLinkPointer_(endEffectorLinkName);

        // Get a pointer to the cup link        
        std::string cupLinkName =
            static_cast<const AudioClassificationInitialBeliefOptions *>(options_.get())->cupLink;
        cupLink_ = getLinkPointer_(cupLinkName);

        if (!endEffectorLink_)
            ERROR("End effector link '" + endEffectorLinkName + "' couldn't be found");
        if (!cupLink_)
            ERROR("Cup link '" + cupLinkName + "' couldn't be found");        
        return true;
    }

    virtual RobotStateSharedPtr sampleAnInitState() override {
        // First, force a full reset of the gazebo world
        auto world = robotEnvironment_->getGazeboInterface()->getWorld();
        world->Reset();
        world->ResetPhysicsStates();
        world->ResetTime();

        // This will set the joint angles to the ones defined in initialStateVector_
        robotEnvironment_->getGazeboInterface()->setStateVector(initialStateVector_);

        // Now compute the cup pose in world coordinates
        GZPose endEffectorPose = endEffectorLink_->WorldPose();

        // Relative pose of the cup with respect to the end effector
        GZPose relativeCupPose;
        relativeCupPose.Pos().X() = initialStateVector_[7];
        relativeCupPose.Pos().Y() = initialStateVector_[8];
        relativeCupPose.Pos().Z() = initialStateVector_[9];

        // The cup pose in world coordinates
        GZPose cupWorldPoseGZ = relativeCupPose + endEffectorPose;
        geometric::Pose cupWorldPose(cupWorldPoseGZ);

        cupLink_->SetWorldPose(cupWorldPoseGZ);

        // This is only necessary if we want the viewer to correctly reflect the initial state
        EnvironmentChangeSharedPtr environmentChange(new BodyPoseChange(cupLink_->GetScopedName(), cupWorldPose));
        robotEnvironment_->addEnvironmentChange(environmentChange);
        robotEnvironment_->applyChanges();

        // Now that we've set the initial joint angles and initial cup pose, construct a gazebo world state
        robotEnvironment_->getGazeboInterface()->makeInitialWorldState(initialStateVector_, false);

        // Then get the initial world state
        GazeboWorldStatePtr initialWorldState = robotEnvironment_->getGazeboInterface()->getInitialWorldState();

        // Construct the initial state
        RobotStateSharedPtr initialState(new VectorState(initialStateVector_));
        initialState->setGazeboWorldState(initialWorldState);
        initialState->setUserData(makeUserData_());
        return initialState;
    }

private:
    RobotEnvironment* robotEnvironment_ = nullptr;

    VectorFloat initialStateVector_;

    // Pointer to the end effector link
    gazebo::physics::Link *endEffectorLink_ = nullptr;

    // Pointer to the cup link
    gazebo::physics::Link *cupLink_ = nullptr;

private:
    OpptUserDataSharedPtr makeUserData_() const {
        OpptUserDataSharedPtr userData(new AudioClassificationUserData);
        userData->as<AudioClassificationUserData>()->endEffectorPose = geometric::Pose(endEffectorLink_->WorldPose());
        return userData;
    }

    gazebo::physics::Link *getLinkPointer_(const std::string &linkName) const {
        gazebo::physics::Link *linkPtr = nullptr;
        auto links = robotEnvironment_->getGazeboInterface()->getLinks();        
        for (auto &link : links) {
            if (link->GetScopedName().find(linkName) != std::string::npos) {
                linkPtr = link;
                break;
            }
        }

        return linkPtr;
    }
};

OPPT_REGISTER_INITIAL_BELIEF_PLUGIN(AudioClassificationInitialBeliefPlugin)

}

#endif
