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

namespace oppt
{
class AudioClassificationObservationPlugin: public ObservationPlugin
{
public :
    AudioClassificationObservationPlugin():
        ObservationPlugin() {

    }

    virtual ~AudioClassificationObservationPlugin() = default;

    virtual bool load(RobotEnvironment* const robotEnvironment, const std::string& optionsFile) override {
        robotEnvironment_ = robotEnvironment;        
        return true;
    }

    virtual ObservationResultSharedPtr getObservation(const ObservationRequest* observationRequest) const override {
        ObservationResultSharedPtr observationResult(new ObservationResult);
        VectorFloat observationVec({0.0});
        observationResult->observation = ObservationSharedPtr(new VectorObservation(observationVec));
        return observationResult;
    }

    virtual double calcLikelihood(const RobotStateSharedPtr &state,
                                  const Action *action,
                                  const Observation *observation) const override {
        unsigned int binNumber = action->as<DiscreteVectorAction>()->getBinNumber();
        if (binNumber == 4) {
            // SLIDE action
        } else if (binNumber == 5) {
            // BANG action
        }
        return 1.0;
    }   

private:
    const RobotEnvironment* robotEnvironment_;
};

OPPT_REGISTER_OBSERVATION_PLUGIN(AudioClassificationObservationPlugin)

}
