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
#include <chrono>
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

    virtual bool load(const std::string& optionsFile) override {
        nodeHandle_ = std::make_unique<ros::NodeHandle>();
        makeObservationDistribution();
        serviceClient_ =
            std::make_unique<ros::ServiceClient>((*(nodeHandle_.get())).serviceClient<ObservationService::Observation>("Observations"));
        return true;
    }

    virtual ObservationResultSharedPtr getObservation(const ObservationRequest* observationRequest) const override {
        int x = 1;
        //ros::NodeHandle nodeHandle_;


        ObservationResultSharedPtr observationResult(new ObservationResult);
        VectorFloat observationVec({0.0, 0.0});
        observationResult->state = observationRequest->currentState.get();
        observationResult->action = observationRequest->action;
        auto Action = observationRequest->action;
        unsigned int binNumber = Action->as<DiscreteVectorAction>()->getBinNumber();
        VectorFloat stateVec = observationRequest->currentState->as<VectorState>()->asVector();
        FloatType sample = dist_(*(robotEnvironment_->getRobot()->getRandomEngine().get()));
        // std::this_thread::sleep_for(std::chrono::seconds(2));
        cout<<"Press enter to get observation : "<<endl;
        getchar();
        FloatType centroid = 0.0;
        FloatType rms = 0.0;
        if (binNumber == 4) //SLIDE
        {
            if (stateVec[stateVec.size() - 1] == 0) // PLASTIC CUP
            {
                ObservationService::Observation srv;
                srv.request.state = atoll("0");
                srv.request.action = atoll("0");                
                if ((*(serviceClient_.get())).call(srv))
                {
                    centroid = (FloatType)srv.response.centroid;
                    rms = (FloatType)srv.response.rms;
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
                if ((*(serviceClient_.get())).call(srv))
                {
                    centroid = (FloatType)srv.response.centroid;
                    rms = (FloatType)srv.response.rms;
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
                if ((*(serviceClient_.get())).call(srv))
                {
                    centroid = (FloatType)srv.response.centroid;
                    rms = (FloatType)srv.response.rms;
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
                srv.request.state = atoll("1");
                srv.request.action = atoll("1");
                if ((*(serviceClient_.get())).call(srv))
                {
                    centroid = (FloatType)srv.response.centroid;
                    rms = (FloatType)srv.response.rms;
                }

                // if (sample <= 0.7)
                //     observationVec[0] = 0;
                // else if (sample <= 0.9)
                //     observationVec[0] = 1;
                // else
                //     observationVec[0] = 2;
            }
        }
        observationVec[0] = centroid;
        observationVec[1] = rms;
        cout<<"Observation received" << observationVec[0] << "  "<<observationVec[1]<<endl;
        observationResult->observation = ObservationSharedPtr(new VectorObservation(observationVec));
        return observationResult;
    }

    virtual double calcLikelihood(const RobotStateSharedPtr &state,
                                  const Action *action,
                                  const Observation *observation) const override {
        unsigned int binNumber = action->as<DiscreteVectorAction>()->getBinNumber();
        VectorFloat stateVec = state->as<VectorState>()->asVector();
        VectorFloat observationVec = observation->as<VectorObservation>()->asVector();
        FloatType pdf = 0.0;
        if (binNumber == 4) // SLIDE ACTION
        {
            if (stateVec[stateVec.size() - 1] == 0) //PLASTIC CUP
            {
                pdf = pringles_can_action1->pdf(observationVec);
            }
            else // COFFEE MUG
            {
                pdf = coffee_mug_action1->pdf(observationVec);
            }
        }
        else if (binNumber == 5) //BANG ACTION
        {
            if (stateVec[stateVec.size() - 1] == 0) //PLASTIC CUP
            {
                pdf = pringles_can_action2->pdf(observationVec);
            }
            else //COFFEE MUG
            {
                pdf = coffee_mug_action2->pdf(observationVec);
                
            }
        }
        else
        {
            if (observationVec[observationVec.size() - 1] == 0.0 && observationVec[observationVec.size() - 2] == 0.0)
                return 1.0;
            else
                cout<<observationVec[0]<<"  "<<observationVec[1]<<endl;
                ERROR("IMPOSSIBLE");
        }

        // cout<<binNumber<<"  "<< stateVec[stateVec.size() - 1]<<"  "<< pdf << endl;
        // cout<<observationVec[0]<<"   "<<observationVec[1]<<endl;

        return pdf;

    }

    bool makeObservationDistribution() {
        auto randomEngine = robotEnvironment_->getRobot()->getRandomEngine();


        coffee_mug_action1 = std::unique_ptr<MultivariateNormalDistribution<FloatType>>(new MultivariateNormalDistribution<FloatType>(randomEngine));
        coffee_mug_action2 = std::unique_ptr<MultivariateNormalDistribution<FloatType>>(new MultivariateNormalDistribution<FloatType>(randomEngine));
        pringles_can_action1 = std::unique_ptr<MultivariateNormalDistribution<FloatType>>(new MultivariateNormalDistribution<FloatType>(randomEngine));
        pringles_can_action2 = std::unique_ptr<MultivariateNormalDistribution<FloatType>>(new MultivariateNormalDistribution<FloatType>(randomEngine));

        Matrixdf mean = Matrixdf::Zero(2, 1);
        Matrixdf covarianceMatrix = Matrixdf::Identity(2, 2);
        mean(0,0) = 1387.0;
        mean(1,0) = 0.0155;
        covarianceMatrix(0,0) = 295.0*295.0;
        covarianceMatrix(1,1) = 1.0965e-6;
        coffee_mug_action1->as<MultivariateNormalDistribution<FloatType>>()->setMean(mean);
        coffee_mug_action1->as<MultivariateNormalDistribution<FloatType>>()->setCovariance(covarianceMatrix);

        mean = Matrixdf::Zero(2, 1);
        covarianceMatrix = Matrixdf::Identity(2, 2);
        mean(0,0) = 1254.0;
        mean(1,0) = 0.01963;
        covarianceMatrix(0,0) = 98.0*98.0;
        covarianceMatrix(1,1) = 1.4e-6;
        coffee_mug_action2->as<MultivariateNormalDistribution<FloatType>>()->setMean(mean);
        coffee_mug_action2->as<MultivariateNormalDistribution<FloatType>>()->setCovariance(covarianceMatrix);

        mean = Matrixdf::Zero(2, 1);
        covarianceMatrix = Matrixdf::Identity(2, 2);
        mean(0,0) = 1424.0;
        mean(1,0) = 0.01452;
        covarianceMatrix(0,0) = 174.0*174.0;
        covarianceMatrix(1,1) = 4.26e-6;
        pringles_can_action1->as<MultivariateNormalDistribution<FloatType>>()->setMean(mean);
        pringles_can_action1->as<MultivariateNormalDistribution<FloatType>>()->setCovariance(covarianceMatrix);

        mean = Matrixdf::Zero(2, 1);
        covarianceMatrix = Matrixdf::Identity(2, 2);
        mean(0,0) = 1993.0;
        mean(1,0) = 0.01597;
        covarianceMatrix(0,0) = 159.0*159.0;
        covarianceMatrix(1,1) = 4.53e-6;
        pringles_can_action2->as<MultivariateNormalDistribution<FloatType>>()->setMean(mean);
        pringles_can_action2->as<MultivariateNormalDistribution<FloatType>>()->setCovariance(covarianceMatrix);
    }

private:
    mutable std::uniform_real_distribution<FloatType> dist_;

    std::unique_ptr<ros::NodeHandle> nodeHandle_ = nullptr;

    std::unique_ptr<ros::ServiceClient> serviceClient_ = nullptr;

    std::unique_ptr<Distribution<FloatType>> coffee_mug_action1;
    std::unique_ptr<Distribution<FloatType>> coffee_mug_action2;
    std::unique_ptr<Distribution<FloatType>> pringles_can_action1;
    std::unique_ptr<Distribution<FloatType>> pringles_can_action2;
};

OPPT_REGISTER_OBSERVATION_PLUGIN(AudioClassificationObservationPluginExecution)

}
