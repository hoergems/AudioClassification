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

#include "ros/ros.h"
// #include "/home/jihirshu/workspaces/ObjectDetection_ws/devel/include/ObservationService/Observation.h"
// #include "/home/jihirshu/workspaces/ObjectDetection_ws/devel/include/ObservationService/ObservationRequest.h"
// #include "/home/jihirshu/workspaces/ObjectDetection_ws/devel/include/ObservationService/ObservationResponse.h"
#include "Observation.h"
#include "ObservationRequest.h"
#include "ObservationResponse.h"
#include <oppt/plugin/Plugin.hpp>
#include "oppt/opptCore/Distribution.hpp"
namespace oppt
{
class AudioClassificationObservationPluginExecution: public ObservationPlugin
{
public :
    AudioClassificationObservationPluginExecution():
        ObservationPlugin() {

    }

    virtual ~AudioClassificationObservationPluginExecution() = default;

    virtual bool load(RobotEnvironment* const robotEnvironment, const std::string& optionsFile) override {
        robotEnvironment_ = robotEnvironment;        
        return true;
    }

    virtual ObservationResultSharedPtr getObservation(const ObservationRequest* observationRequest) const override {

        char **argv  = nullptr;
        int x = 1;
        ros::NodeHandle n;
        ros::ServiceClient client = n.serviceClient<ObservationService::Observation>("Observations");

        ObservationResultSharedPtr observationResult(new ObservationResult);
        VectorFloat observationVec({0.0});
        observationResult->state = observationRequest->currentState.get();
        observationResult->action = observationRequest->action;
        auto Action = observationRequest->action;
        unsigned int binNumber = Action->as<DiscreteVectorAction>()->getBinNumber();
        VectorFloat stateVec = observationRequest->currentState->as<VectorState>()->asVector();
        FloatType sample = dist_(*(robotEnvironment_->getRobot()->getRandomEngine().get()));

        int response = 3;
        if (binNumber == 4) //SLIDE
        {
            if (stateVec[stateVec.size() - 1] == 0) // PLASTIC CUP
            {
                ObservationService::Observation srv;
                srv.request.state = atoll("0");
                srv.request.action = atoll("0");
                if (client.call(srv))
                {
                    response = (int)srv.response.observation;
                    
                }

                // if (sample <= 0.1)
                //     observationVec[0] = 0;
                // else if (sample <= 0.2)
                //     observationVec[0] = 1;
                // else
                //     observationVec[0] = 2;
            }
            else // COFFEE MUG
            {
                ObservationService::Observation srv;
                srv.request.state = atoll("1");
                srv.request.action = atoll("0");
                if (client.call(srv))
                {
                    response = (int)srv.response.observation;
                    
                }

                // if (sample <= 0.1)
                //     observationVec[0] = 0;
                // else if (sample <= 0.8)
                //     observationVec[0] = 1;
                // else
                //     observationVec[0] = 2;
            }
        }
        else if (binNumber == 5) //BANG
        {
            if (stateVec[stateVec.size() - 1] == 0) // PLASTIC CUP
            {
                ObservationService::Observation srv;
                srv.request.state = atoll("0");
                srv.request.action = atoll("1");
                if (client.call(srv))
                {
                    response = (int)srv.response.observation;
                    
                }

                // if (sample <= 0.1)
                //     observationVec[0] = 0;
                // else if (sample <= 0.8)
                //     observationVec[0] = 1;
                // else
                //     observationVec[0] = 2;

            }
            else //COFFEE MUG
            {
                ObservationService::Observation srv;
                srv.request.state = atoll("0");
                srv.request.action = atoll("0");
                if (client.call(srv))
                {
                    response = (int)srv.response.observation;
                    
                }

                // if (sample <= 0.7)
                //     observationVec[0] = 0;
                // else if (sample <= 0.9)
                //     observationVec[0] = 1;
                // else
                //     observationVec[0] = 2;
            }
        }
        observationVec[0] = response;

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
    const RobotEnvironment* robotEnvironment_;

    mutable std::uniform_real_distribution<FloatType> dist_;
};

OPPT_REGISTER_OBSERVATION_PLUGIN(AudioClassificationObservationPluginExecution)

}
