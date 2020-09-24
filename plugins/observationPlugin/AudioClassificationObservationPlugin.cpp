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
        makeObservationDistribution();
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
                Vectordf obs = pringles_can_action1->sample(1);
                observationVec[0] = obs(0,0);
                // cout<<"Pringles can action 1 : "<<observationVec[0]<<endl;
            }
            else // COFFEE MUG
            {
                Vectordf obs = coffee_mug_action1->sample(1);
                observationVec[0] = obs(0,0);
                // cout<<"Coffee mug action 1 : "<<observationVec[0]<<endl;
            }
            
        }
        else if (binNumber == 5) //BANG
        {
            if (stateVec[stateVec.size() - 1] == 0) // PLASTIC CUP
            {
                Vectordf obs = pringles_can_action2->sample(1);
                observationVec[0] = obs(0,0);
                // cout<<"Pringles can action 2 : "<<observationVec[0]<<endl;   
            }
            else //COFFEE MUG
            {
                Vectordf obs = coffee_mug_action2->sample(1);
                observationVec[0] = obs(0,0);
                // cout<<"Coffee mug action 2 : "<<observationVec[0]<<endl;
            }

        }
        else
        {   
            observationVec[0] = 0;
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
        FloatType pdf = 0.0;
        if (binNumber == 4) // SLIDE ACTION
        {
            if (stateVec[stateVec.size() - 1] == 0) //PLASTIC CUP
            {
                pdf = pringles_can_action1->pdf(observationVec);
                // cout<<"Pringles can action 1 PDF : "<<observationVec[0]<<" : "<<pdf<<endl;
            }
            else // COFFEE MUG
            {
                pdf = coffee_mug_action1->pdf(observationVec);
                // cout<<"Coffee mug action 1 PDF : "<<observationVec[0]<<" : "<<pdf<<endl;
            }
        }
        else if (binNumber == 5) //BANG ACTION
        {
            if (stateVec[stateVec.size() - 1] == 0) //PLASTIC CUP
            {
                pdf = pringles_can_action2->pdf(observationVec);
                // cout<<"Pringles can action 2 PDF : "<<observationVec[0]<<" : "<<pdf<<endl;
            }
            else //COFFEE MUG
            {
                pdf = coffee_mug_action2->pdf(observationVec);
                // cout<<"Coffee mug action 2 PDF : "<<observationVec[0]<<" : "<<pdf<<endl;
                
            }
        }
        else
        {
            if (observationVec[observationVec.size() - 1] == 0)
                return 1.0;
            else
                ERROR("IMPOSSIBLE");
        }

        if (pdf < 1e-6) {
            LOGGING("OBSERVATION PROB IS TOO SMALL");
            return 0.0001;
        }

        
        return pdf;
    }   

    bool makeObservationDistribution() 
    {
        auto randomEngine = robotEnvironment_->getRobot()->getRandomEngine();

        coffee_mug_action1 = std::unique_ptr<MultivariateNormalDistribution<FloatType>>(new MultivariateNormalDistribution<FloatType>(randomEngine));
        coffee_mug_action2 = std::unique_ptr<MultivariateNormalDistribution<FloatType>>(new MultivariateNormalDistribution<FloatType>(randomEngine));
        pringles_can_action1 = std::unique_ptr<MultivariateNormalDistribution<FloatType>>(new MultivariateNormalDistribution<FloatType>(randomEngine));
        pringles_can_action2 = std::unique_ptr<MultivariateNormalDistribution<FloatType>>(new MultivariateNormalDistribution<FloatType>(randomEngine));

        Matrixdf mean = Matrixdf::Zero(1, 1);
        Matrixdf covarianceMatrix = Matrixdf::Identity(1, 1);
        mean(0,0) = 1287.0;
        covarianceMatrix(0,0) = 295.0;
        coffee_mug_action1->as<MultivariateNormalDistribution<FloatType>>()->setMean(mean);
        coffee_mug_action1->as<MultivariateNormalDistribution<FloatType>>()->setCovariance(covarianceMatrix);

        mean = Matrixdf::Zero(1, 1);
        covarianceMatrix = Matrixdf::Identity(1, 1);
        mean(0,0) = 1254.0;
        covarianceMatrix(0,0) = 98.0;
        coffee_mug_action2->as<MultivariateNormalDistribution<FloatType>>()->setMean(mean);
        coffee_mug_action2->as<MultivariateNormalDistribution<FloatType>>()->setCovariance(covarianceMatrix);

        mean = Matrixdf::Zero(1, 1);
        covarianceMatrix = Matrixdf::Identity(1, 1);
        mean(0,0) = 1424.0;
        covarianceMatrix(0,0) = 174.0;
        pringles_can_action1->as<MultivariateNormalDistribution<FloatType>>()->setMean(mean);
        pringles_can_action1->as<MultivariateNormalDistribution<FloatType>>()->setCovariance(covarianceMatrix);

        mean = Matrixdf::Zero(1, 1);
        covarianceMatrix = Matrixdf::Identity(1, 1);
        mean(0,0) = 1993.0;
        covarianceMatrix(0,0) = 159.0;
        pringles_can_action2->as<MultivariateNormalDistribution<FloatType>>()->setMean(mean);
        pringles_can_action2->as<MultivariateNormalDistribution<FloatType>>()->setCovariance(covarianceMatrix);

    }

private:
    mutable std::uniform_real_distribution<FloatType> dist_;

    std::unique_ptr<Distribution<FloatType>> coffee_mug_action1;
    std::unique_ptr<Distribution<FloatType>> coffee_mug_action2;
    std::unique_ptr<Distribution<FloatType>> pringles_can_action1;
    std::unique_ptr<Distribution<FloatType>> pringles_can_action2;

};

OPPT_REGISTER_OBSERVATION_PLUGIN(AudioClassificationObservationPlugin)

}
