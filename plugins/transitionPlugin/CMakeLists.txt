set (TRANSITION_PLUGIN_SRC
     ${CMAKE_CURRENT_LIST_DIR}/AudioClassificationTransitionPlugin.cpp
     ${ROOT_PATH}/plugins/shared/MovoRobotInterface/MovoRobotInterface.cpp
     ${ROOT_PATH}/plugins/shared/MovoRobotInterface/HardwareStartup.cpp
     ${ROOT_PATH}/plugins/shared/MovoRobotInterface/MovoAPI.cpp)

ADD_TRANSITION_PLUGIN(audioClassificationTransitionPlugin "${TRANSITION_PLUGIN_SRC}")
target_include_directories(audioClassificationTransitionPlugin PRIVATE ${ROOT_PATH}/plugins/shared/)
