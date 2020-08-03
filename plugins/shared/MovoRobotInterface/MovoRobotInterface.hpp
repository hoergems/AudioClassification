#ifndef _MOVO_ROBOT_INTERFACE_HPP_
#define _MOVO_ROBOT_INTERFACE_HPP_
#include <oppt/opptCore/core.hpp>
#include "MovoAPI.hpp"

namespace oppt {
class MovoRobotInterface {
public:
	MovoRobotInterface() = default;

	~MovoRobotInterface() = default;

	_NO_COPY_BUT_MOVE(MovoRobotInterface)

	/**
	 * @brief Initializes the interface to the robot by performing a hardware startup
	 */
	void init();

	/**
	 * @brief Get the current joint angles of the robot
	 */
	VectorFloat getCurrentJointAngles() const;

	/**
	 * @brief Open the gripper
	 */
	void openGripper();

	/**
	 * @brief Close the gripper
	 */
	void closeGripper();

	/**
	 * @brief Move to initial jointAngles
	 */
	void moveToInitialJointAngles(const VectorFloat &initialJointAngles);

	/**
	 * @brief Apply the joint velocities for a duration given by durationMs (the duration in milliseconds)
	 */
	bool applyJointVelocities(const VectorFloat &jointVelocities, const FloatType &durationMS) const;

private:
	std::unique_ptr<movo::MovoAPI> movoAPI_ = nullptr;

	bool gripperClosed_ = false;

private:
	bool sendTargetJointAngles_(const VectorFloat &jointAngles, const FloatType &duration);

	bool sendTargetJointVelocities_(const VectorFloat &jointVelocities) const;

};
}

#endif