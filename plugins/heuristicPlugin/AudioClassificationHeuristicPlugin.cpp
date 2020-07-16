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
#ifndef _DEFAULT_HEURISTIC_PLUGIN_HPP_
#define _DEFAULT_HEURISTIC_PLUGIN_HPP_
#include <oppt/plugin/Plugin.hpp>

namespace oppt
{
class AudioClassificationHeuristicPlugin: public HeuristicPlugin
{
public:
    AudioClassificationHeuristicPlugin():
        HeuristicPlugin() {

    }

    virtual ~AudioClassificationHeuristicPlugin() = default;

    virtual bool load(RobotEnvironment* const robotEnvironment,
                      const std::string& optionsFile) override {
        //parseOptions_<LightDarkHeuristicOptions>(optionsFile);        
        optionsFile_ = optionsFile;        
        return true;
    }

    virtual double getHeuristicValue(const HeuristicInfo* heuristicInfo) const override {
        return 0.0;        
    }

    virtual HeuristicPluginSharedPtr clone(RobotEnvironment* const robotEnvironment) const override {
        std::shared_ptr<HeuristicPlugin> clonedPlugin(new AudioClassificationHeuristicPlugin());
        clonedPlugin->load(robotEnvironment, optionsFile_);
        return clonedPlugin;
    }

private:
    std::string optionsFile_;

};

OPPT_REGISTER_HEURISTIC_PLUGIN(AudioClassificationHeuristicPlugin)

}

#endif
