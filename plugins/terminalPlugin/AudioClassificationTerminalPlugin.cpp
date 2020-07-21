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
class AudioClassificationTerminalPlugin: public TerminalPlugin
{
public :
    AudioClassificationTerminalPlugin():
        TerminalPlugin() {

    }

    virtual ~AudioClassificationTerminalPlugin() = default;

    virtual bool load(RobotEnvironment* const robotEnvironment, const std::string& optionsFile) override {
        robotEnvironment_ = robotEnvironment;
        return true;
    }

    virtual ValidityReportSharedPtr isValid(const PropagationResultSharedPtr& propagationResult) override {
        ValidityReportSharedPtr validityReport(new ValidityReport(propagationResult->nextState));
        validityReport->isValid = true;
        validityReport->satisfiesConstraints = true;
        validityReport->collided = false;
        return validityReport;
    }

    virtual bool isTerminal(const PropagationResultSharedPtr& propagationResult) override {

        VectorFloat actionVec = propagationResult->action->as<VectorAction>()->asVector();
        unsigned int binNumber = propagationResult->action->as<DiscreteVectorAction>()->getBinNumber();

        if ((binNumber == 6) || (binNumber == 7))
            return true;

        return false;
    }

private:
    const RobotEnvironment* robotEnvironment_;
};

OPPT_REGISTER_TERMINAL_PLUGIN(AudioClassificationTerminalPlugin)

}




