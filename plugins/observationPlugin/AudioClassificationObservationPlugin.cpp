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
#include "oppt/opptCore/Distribution.hpp"
namespace oppt
{
class AudioClassificationObservationPlugin: public ObservationPlugin
{
public :
    AudioClassificationObservationPlugin():
        ObservationPlugin() {

    }

    virtual ~AudioClassificationObservationPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        return true;
    }

    virtual ObservationResultSharedPtr getObservation(const ObservationRequest* observationRequest) const override {
        ObservationResultSharedPtr observationResult(new ObservationResult);
        VectorFloat observationVec({0.0});
        observationResult->state = observationRequest->currentState.get();
        observationResult->action = observationRequest->action;
        auto Action = observationRequest->action;
        unsigned int binNumber = Action->as<DiscreteVectorAction>()->getBinNumber();
        VectorFloat stateVec = observationRequest->currentState->as<VectorState>()->asVector();
        FloatType sample = dist_(*(robotEnvironment_->getRobot()->getRandomEngine().get()));

        if (binNumber == 4) //SLIDE
        {
            if (stateVec[stateVec.size() - 1] == 0) // PLASTIC CUP
            {
                if (sample <= 0.1)
                    observationVec[0] = 0;
                else if (sample <= 0.2)
                    observationVec[0] = 1;
                else
                    observationVec[0] = 2;
            }
            else // COFFEE MUG
            {
                if (sample <= 0.1)
                    observationVec[0] = 0;
                else if (sample <= 0.8)
                    observationVec[0] = 1;
                else
                    observationVec[0] = 2;
            }
        }
        else if (binNumber == 5) //BANG
        {
            if (stateVec[stateVec.size() - 1] == 0) // PLASTIC CUP
            {
                if (sample <= 0.1)
                    observationVec[0] = 0;
                else if (sample <= 0.8)
                    observationVec[0] = 1;
                else
                    observationVec[0] = 2;

            }
            else //COFFEE MUG
            {
                if (sample <= 0.7)
                    observationVec[0] = 0;
                else if (sample <= 0.9)
                    observationVec[0] = 1;
                else
                    observationVec[0] = 2;
            }
        }
        else
        {   
            observationVec[0] = 3;
        }


        observationResult->observation = ObservationSharedPtr(new VectorObservation(observationVec));
        return observationResult;
    }

    virtual double calcLikelihood(const RobotStateSharedPtr &state,
                                  const Action *action,
                                  const Observation *observation) const override {
        unsigned int binNumber = action->as<DiscreteVectorAction>()->getBinNumber();
        VectorFloat stateVec = state->as<VectorState>()->asVector();
        VectorFloat observationVec = observation->as<VectorObservation>()->asVector();
        if (binNumber == 4) // SLIDE ACTION
        {
            if (stateVec[stateVec.size() - 1] == 0) //PLASTIC CUP
            {
                if (observationVec[observationVec.size() - 1] == 2)
                    return 0.8;
                else
                    return 0.1;
            }
            else // COFFEE MUG
            {
                if (observationVec[observationVec.size() - 1] == 1)
                    return 0.7;
                else if (observationVec[observationVec.size() -1] == 0)
                    return 0.1;
                else
                    return 0.2;
            }
        } 
        else if (binNumber == 5) //BANG ACTION
        {
            if (stateVec[stateVec.size() - 1] == 0) //PLASTIC CUP
            {
                if (observationVec[observationVec.size() - 1] == 1)
                    return 0.7;
                else if (observationVec[observationVec.size() - 1] == 0)
                    return 0.1;
                else
                    return 0.2;
            }
            else //COFFEE MUG
            {
                if (observationVec[observationVec.size() - 1] == 0)
                    return 0.7;
                else if (observationVec[observationVec.size() -1] == 1)
                    return 0.2;
                else
                    return 0.1;
            }            
        }
        else
        {
            if (observationVec[observationVec.size() - 1] == 3)
                return 1.0;
            else
                ERROR("IMPOSSIBLE");
        }
    }   

private:
    mutable std::uniform_real_distribution<FloatType> dist_;
};

OPPT_REGISTER_OBSERVATION_PLUGIN(AudioClassificationObservationPlugin)

}
