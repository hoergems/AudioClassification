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
#include <oppt/plugin/Plugin.hpp>
#include <oppt/robotHeaders/InverseKinematics/InverseKinematics.hpp>
#include <oppt/gazeboInterface/GazeboInterface.hpp>
#include "AudioClassificationActionDiscretizer.hpp"
#include "AudioClassificationTransitionPluginOptions.hpp"
#include "AudioClassificationUserData.hpp"

namespace oppt
{

class AudioClassificationTransitionPlugin: public TransitionPlugin
{
public :
    AudioClassificationTransitionPlugin():
        TransitionPlugin() {

    }

    virtual ~AudioClassificationTransitionPlugin() = default;

    virtual bool load(RobotEnvironment* const robotEnvironment, const std::string& optionsFile) override {
        robotEnvironment_ = robotEnvironment;
        parseOptions_<AudioClassificationTransitionPluginOptions>(optionsFile);
        auto options = static_cast<const AudioClassificationTransitionPluginOptions *>(options_.get());
        auto actionSpace = robotEnvironment->getRobot()->getActionSpace();

        // Setup a custom action discretizer.
        // This action discretizer will generate 8 actions. See AudioClassificationActionDiscretizer.hpp
        std::shared_ptr<ActionSpaceDiscretizer> robotActionDiscretizer =
            std::shared_ptr<ActionSpaceDiscretizer>(new AudioClassificationActionDiscretizer(actionSpace,
                    std::vector<unsigned int>()));
        actionSpace->setActionSpaceDiscretizer(robotActionDiscretizer);

        // Get a pointer to the end-effector link
        std::string endEffectorLinkName = options->endEffectorLink;
        endEffectorLink_ = getLinkPointer_(endEffectorLinkName);
        if (!endEffectorLink_)
            ERROR("End effector link '" + endEffectorLinkName + "' couldn't be found");

        // Get a pointer to the cup link
        std::string cupLinkName = options->cupLink;
        cupLink_ = getLinkPointer_(cupLinkName);
        if (!cupLink_)
            ERROR("Cup link '" + cupLinkName + "' couldn't be found");

        // Setup the ik solver
        setupIKSolver_();

        endEffectorMotionDistance_ = options->endEffectorMotionDistance;
        return true;
    }

    virtual PropagationResultSharedPtr propagateState(const PropagationRequest* propagationRequest) const override {
        // First update Gazebo with the world state contained in the current state
        robotEnvironment_->getGazeboInterface()->setWorldState(propagationRequest->currentState->getGazeboWorldState().get());
        VectorFloat currentStateVector = propagationRequest->currentState->as<VectorState>()->asVector();
        VectorFloat endEffectorVelocity(6, 0.0);

        unsigned int actionBinNumber = propagationRequest->action->as<DiscreteVectorAction>()->getBinNumber();

        // Determines if, and which macro action is being executed.
        // 0 = no macro action
        // 1 = slide action
        // 2 = bang action
        // 3 = move cup to location A
        // 4 = move cup to location B
        int macroAction = 0;

        switch (actionBinNumber) {
        case 0:
            // X_PLUS
            endEffectorVelocity[0] += endEffectorMotionDistance_;
            break;
        case 1:
            // X_MINUS
            endEffectorVelocity[0] -= endEffectorMotionDistance_;
            break;
        case 2:
            // Y_PLUS
            endEffectorVelocity[1] += endEffectorMotionDistance_;
            break;
        case 3:
            // Y_MINUS
            endEffectorVelocity[1] -= endEffectorMotionDistance_;
            break;
        case 4:
            // SLIDE
            macroAction = 1;
            break;
        case 5:
            macroAction = 2;
            // BANG
            break;
        case 6:
            // Move location A
            macroAction = 3;
            break;
        case 7:
            // Move location B
            macroAction = 4;
            break;
        default:
            ERROR("Action not recognized");
            break;
        }

        VectorFloat nextJointAngles;
        if (macroAction != 0) {
            // A macro action is being executed
            nextJointAngles =
                performMacroAction_(currentStateVector,
                                    propagationRequest->currentState->getUserData()->as<AudioClassificationUserData>()->endEffectorPose,
                                    macroAction);

        } else {
            // One of the end effector motion actions (X_PLUS, X_MINUS, Y_PLUS, Y_MINUS) is being executed
            nextJointAngles = applyEndEffectorVelocity_(currentStateVector, endEffectorVelocity);
        }

        // The first 7 dimensions of the next state vector are the new joint angles
        VectorFloat nextStateVector = nextJointAngles;

        // REMOVE ME
        LOGGING("Should run fine");
        getchar();

        // The next 3 dimensions of the next state vector describe the relative position of
        // the cup with respect to the end effector
        nextStateVector.push_back(cupLink_->WorldPose().Pos().X() - endEffectorLink_->WorldPose().Pos().X());
        nextStateVector.push_back(cupLink_->WorldPose().Pos().Y() - endEffectorLink_->WorldPose().Pos().Y());
        nextStateVector.push_back(cupLink_->WorldPose().Pos().Z() - endEffectorLink_->WorldPose().Pos().Z());

        PropagationResultSharedPtr propagationResult(new PropagationResult);
        propagationResult->nextState = RobotStateSharedPtr(new VectorState(nextStateVector));
        propagationResult->nextState->setUserData(makeUserData_());
        propagationResult->nextState->setGazeboWorldState(robotEnvironment_->getGazeboInterface()->getWorldState(true));
        return propagationResult;
    }

private:
    const RobotEnvironment* robotEnvironment_;

    /** @brief A pointer to the end effector link */
    gazebo::physics::Link *endEffectorLink_ = nullptr;

    /** @brief A pointer to the cup link */
    gazebo::physics::Link *cupLink_ = nullptr;

    /**
     * @brief The default motion distance of the end-effector (in meters) for each directional action
     * This distance can be modified in the configuration file (via transitionPluginOptions.endEffectorMotionDistance)
     */
    FloatType endEffectorMotionDistance_ = 0.05;

private:

    VectorFloat applyEndEffectorVelocity_(const VectorFloat &currentStateVector, const VectorFloat &endEffectorVelocity) const {
        auto tracIkSolver = static_cast<oppt::TracIKSolver *>(robotEnvironment_->getRobot()->getIKSolver());
        VectorFloat currentJointAngles(currentStateVector.begin(), currentStateVector.begin() + 7);

        // Get the vector of joint velocities corresponding to the end effector velocity
        VectorFloat jointVelocities =
            tracIkSolver->jointVelocitiesFromTwist(currentJointAngles, endEffectorVelocity);

        // The next joint angles are then simply the current joint angles, plus the joint velocities
        VectorFloat newJointAngles = addVectors(currentJointAngles, jointVelocities);

        // Update the Gazebo model with the next joint angles
        robotEnvironment_->getGazeboInterface()->setStateVector(newJointAngles);

        // Check if the cup is inside the gripper. If yes, move the cup with the gripper
        VectorFloat relativeCupPosition({currentStateVector[7],
                                         currentStateVector[8],
                                         currentStateVector[9]
                                        });

        // Here we assume that if the euclidean distance between the end effector and the cup
        // is smaller than 0.01 meters, the cup will "travel" with the end effector.
        // TODO: Make sure the cup remains in its current positions when the end effector
        // moves away from the cup
        FloatType l2norm = math::l2norm(relativeCupPosition);
        if (l2norm < 0.01) {
            GZPose endEffectorPose = endEffectorLink_->WorldPose();
            geometric::Pose newCupPose(endEffectorPose.Pos().X(),
                                       endEffectorPose.Pos().Y(),
                                       endEffectorPose.Pos().Z(),
                                       0.0,
                                       0.0,
                                       0.0);
            cupLink_->SetWorldPose(newCupPose.toGZPose());
        }

        return newJointAngles;
    }

    /**
     * @brief Perform one of the macro actions (SLIDE, BANG, Move to location A, Move to location B)
     */
    VectorFloat performMacroAction_(const VectorFloat &currentStateVector,
                                    const geometric::Pose &currentEndEffectorPose,
                                    const int &macroAction) const {
        VectorFloat newJointAngles;

        if (macroAction == 1 or macroAction == 2) {
            // In case we execute the SLIDE or BANG macro action, we first have to move the
            // end effector to the current cup position
            newJointAngles = moveEndEffectorToCupPosition_(currentStateVector, currentEndEffectorPose);
        }

        if (macroAction == 1) {
            // Afer moving the end effector to the cup position, we additional push the cup by a predefined distance
            // specified by endEffectorMotionDistance_

            // Get the updated end effector pose after we moved it to the cup position
            geometric::Pose newEndEffectorPose(endEffectorLink_->WorldPose());

            // Then we push the cup forward in x-direction (relative to the end effector link)
            geometric::Pose pushVector(endEffectorMotionDistance_, 0.0, 0.0, 0.0, 0.0, 0.0);

            // Compute the desired end effector pose after pushing the cup
            geometric::Pose newEndEffectorWorldPoseAfterPushing = pushVector + newEndEffectorPose;

            // The end effector velocity necessary to bring the end effector from its current pose (the pose after
            // moving it to the cup position) to the desired end effector pose after pushing
            VectorFloat endEffectorVelocity(3, 0.0);
            endEffectorVelocity[0] = newEndEffectorWorldPoseAfterPushing.position.x() - newEndEffectorPose.position.x();
            endEffectorVelocity[1] = newEndEffectorWorldPoseAfterPushing.position.y() - newEndEffectorPose.position.y();
            endEffectorVelocity[2] = newEndEffectorWorldPoseAfterPushing.position.z() - newEndEffectorPose.position.z();

            // Apply this velocity to the end effector. This will result in a new set of joint angles that correspong
            // to the resulting end effector pose (after pushing the object)
            newJointAngles = applyEndEffectorVelocity_(newJointAngles, endEffectorVelocity);

            // We simply set the resulting world pose of the cup (after pushing it) to be equal to the resulting
            // end effector world pose
            geometric::Pose newCupWorldPoseAfterPushing(newEndEffectorWorldPoseAfterPushing.position.x(),
                    newEndEffectorWorldPoseAfterPushing.position.y(),
                    newEndEffectorWorldPoseAfterPushing.position.z(),
                    0.0,
                    0.0,
                    0.0);

            cupLink_->SetWorldPose(newCupWorldPoseAfterPushing.toGZPose());
        }

        if (macroAction == 2) {
            // For the BANG action we assume that the resulting joint angles are equal
            // to the ones we got after moving the end effector to the cup position.
            // So we actually don't have to do anything here
        }

        if (macroAction == 3 or macroAction == 4) {
            // For the Move to location A and Move to location B macro actions,
            // we can (for the moment) simply return the current joint angles,
            // because executing those actions will result in a terminal state
            newJointAngles = VectorFloat(currentStateVector.begin(), currentStateVector.begin() + 7);
        }

        return newJointAngles;
    }


    /**
     * @brief Moves the end effector to the current cup position
     */
    VectorFloat moveEndEffectorToCupPosition_(const VectorFloat & currentStateVector,
            const geometric::Pose & currentEndEffectorPose) const {
        // Get the relative XYZ-position of the cup (with respect to the end effector) from the current state
        VectorFloat currentCupPoseVector(6, 0.0);
        currentCupPoseVector[0] = currentStateVector[7];
        currentCupPoseVector[1] = currentStateVector[8];
        currentCupPoseVector[2] = currentStateVector[9];
        geometric::Pose currentCupRelativePose(currentCupPoseVector[0],
                                               currentCupPoseVector[1],
                                               currentCupPoseVector[2],
                                               currentCupPoseVector[3],
                                               currentCupPoseVector[4],
                                               currentCupPoseVector[5]);

        // Compute the current world pose of the cup
        geometric::Pose currentCupWorldPose = currentCupRelativePose + currentEndEffectorPose;

        // Compute the desired end-effector velocity as the difference between the current world pose of the cup
        // and the current world pose of the end effector
        VectorFloat endEffectorVelocity({currentCupWorldPose.position.x() - currentEndEffectorPose.position.x(),
                                         currentCupWorldPose.position.y() - currentEndEffectorPose.position.y(),
                                         currentCupWorldPose.position.z() - currentEndEffectorPose.position.z(),
                                         0.0,
                                         0.0,
                                         0.0
                                        });

        // Call the applyEndEffectorVelocity_ method to get the set of joint angles that results from
        // applying the end effector velocity to the current pose of the end effector
        return applyEndEffectorVelocity_(currentStateVector, endEffectorVelocity);
    }

    /**
     * @brief Initialized the inverse kinematics solver
     */
    void setupIKSolver_() {
        auto options = static_cast<const AudioClassificationTransitionPluginOptions *>(options_.get());
        std::string urdfFile = options->urdfFile;
        std::string baseLink = options->baseLink;
        std::string endEffectorLink = options->endEffectorLink;

        if (oppt::resources::FileExists(urdfFile) == false)
            ERROR("URDF file '" + urdfFile + "' doesn't exist");

        std::string urdfPath = oppt::resources::FindFile(urdfFile);
        auto randomEngine = robotEnvironment_->getRobot()->getRandomEngine();
        IKSolverUniquePtr ikSolver =
            std::make_unique<TracIKSolver>(randomEngine.get(), urdfPath, baseLink, endEffectorLink);
        auto tracIkSolver = static_cast<oppt::TracIKSolver *>(ikSolver.get());
        if (tracIkSolver->init() == false)
            ERROR("IKSolver could not be initialized");
        robotEnvironment_->getRobot()->setIKSolver(std::move(ikSolver));
    }

    OpptUserDataSharedPtr makeUserData_() const {
        OpptUserDataSharedPtr userData(new AudioClassificationUserData);
        userData->as<AudioClassificationUserData>()->endEffectorPose = geometric::Pose(endEffectorLink_->WorldPose());
        return userData;

    }

    /**
     * @brief Helper function to get a pointer to a link with the given link name
     */
    gazebo::physics::Link *getLinkPointer_(const std::string & linkName) const {
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

OPPT_REGISTER_TRANSITION_PLUGIN(AudioClassificationTransitionPlugin)

}
