#ifndef _MOVO_UTILS_HPP_
#define _MOVO_UTILS_HPP_
#include <oppt/opptCore/core.hpp>
#include <KinovaTypes.h>

namespace oppt {
inline VectorFloat toStandardVec(const AngularInfo &angularInfo) {
	VectorFloat vec;
	if (NUM_JOINTS == 6) {
		vec = VectorFloat({angularInfo.Actuator1,
		                   angularInfo.Actuator2,
		                   angularInfo.Actuator3,
		                   angularInfo.Actuator4,
		                   angularInfo.Actuator5,
		                   angularInfo.Actuator6
		                  });
	} else {
		vec = VectorFloat({angularInfo.Actuator1,
		                   angularInfo.Actuator2,
		                   angularInfo.Actuator3,
		                   angularInfo.Actuator4,
		                   angularInfo.Actuator5,
		                   angularInfo.Actuator6,
		                   angularInfo.Actuator7
		                  });
	}

	return vec;
}

inline AngularInfo toAngularInfo(const VectorFloat &jointAngles) {
	AngularInfo info;
	info.InitStruct();
	if (NUM_JOINTS == 6) {
		info.Actuator1 = jointAngles[0];
		info.Actuator2 = jointAngles[1];
		info.Actuator3 = jointAngles[2];
		info.Actuator4 = jointAngles[3];
		info.Actuator5 = jointAngles[4];
		info.Actuator6 = jointAngles[5];
	} else {
		info.Actuator1 = jointAngles[0];
		info.Actuator2 = jointAngles[1];
		info.Actuator3 = jointAngles[2];
		info.Actuator4 = jointAngles[3];
		info.Actuator5 = jointAngles[4];
		info.Actuator6 = jointAngles[5];
		info.Actuator7 = jointAngles[6];
	}
	return info;
}

inline VectorFloat toDegreesPosition(const VectorFloat &jointAngles) {
	if (jointAngles.size() != NUM_JOINTS)
		ERROR("Joint angles has wrong format");
	VectorFloat resJointAngles(NUM_JOINTS, 0.0);
	if (NUM_JOINTS == 6) {
		resJointAngles[0] = jointAngles[0] * 180.0 / M_PI;
		resJointAngles[1] = (jointAngles[1] * 180.0 / M_PI) + 180.0;
		resJointAngles[2] = (jointAngles[2] * 180.0 / M_PI) + 180.0;
		resJointAngles[3] = jointAngles[3] * 180.0 / M_PI;
		resJointAngles[4] = jointAngles[4] * 180.0 / M_PI;
		resJointAngles[5] = jointAngles[5] * 180.0 / M_PI;
	} else {
		resJointAngles[0] = jointAngles[0] * 180.0 / M_PI;
		resJointAngles[1] = (jointAngles[1] * 180.0 / M_PI) + 180.0;
		resJointAngles[2] = jointAngles[2] * 180.0 / M_PI;
		resJointAngles[3] = (jointAngles[3] * 180.0 / M_PI) + 180.0;
		resJointAngles[4] = jointAngles[4] * 180.0 / M_PI;
		resJointAngles[5] = (jointAngles[5] * 180.0 / M_PI) + 180.0;
		resJointAngles[6] = jointAngles[6] * 180.0 / M_PI;
	}
	return resJointAngles;
}

inline VectorFloat toDegreesVelocity(const VectorFloat &velocity) {
	if (velocity.size() != NUM_JOINTS) {
		cout << "velocity.size(): " << velocity.size() << endl;
		ERROR("Velocity has wrong format");
	}
	VectorFloat res(NUM_JOINTS, 0.0);
	for (size_t i = 0; i != NUM_JOINTS; ++i) {
		res[i] = velocity[i] * 180.0 / M_PI;
	}

	return res;
}

inline VectorFloat toRadianPosition(const VectorFloat &jointAngles) {
	if (jointAngles.size() != NUM_JOINTS)
		ERROR("Joint angles has wrong format");
	VectorFloat resJointAngles(NUM_JOINTS, 0.0);
	if (NUM_JOINTS == 6) {
		resJointAngles[0] = jointAngles[0] * M_PI / 180.0;
		resJointAngles[1] = (jointAngles[1] - 180.0) * M_PI / 180.0;
		resJointAngles[2] = (jointAngles[2] - 180.0) * M_PI / 180.0;
		resJointAngles[3] = jointAngles[3] * M_PI / 180.0;
		resJointAngles[4] = jointAngles[4] * M_PI / 180.0;
		resJointAngles[5] = jointAngles[5] * M_PI / 180.0;
	} else {
		resJointAngles[0] = jointAngles[0] * M_PI / 180.0;
		resJointAngles[1] = (jointAngles[1] - 180.0) * M_PI / 180.0;
		resJointAngles[2] = jointAngles[2] * M_PI / 180.0;
		resJointAngles[3] = (jointAngles[3] - 180.0) * M_PI / 180.0;
		resJointAngles[4] = jointAngles[4] * M_PI / 180.0;
		resJointAngles[5] = (jointAngles[5] - 180.0) * M_PI / 180.0;
		resJointAngles[6] = jointAngles[6] * M_PI / 180.0;
	}

	return resJointAngles;
}

inline VectorFloat toRadianVelocity(const VectorFloat &velocity) {
	if (velocity.size() != NUM_JOINTS)
		ERROR("Velocity has wrong format");
	VectorFloat res(NUM_JOINTS, 0.0);
	for (size_t i = 0; i != NUM_JOINTS; ++i) {
		res[i] = velocity[i] * M_PI / 180.0;
	}

	return res;
}
}

#endif