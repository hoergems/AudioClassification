SUBDIRLIST(SUBDIRS ${CMAKE_CURRENT_LIST_DIR})
FOREACH(subdir ${SUBDIRS})
  if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/${subdir}/CMakeLists.txt")
    include(${CMAKE_CURRENT_LIST_DIR}/${subdir}/CMakeLists.txt)
  endif()  
ENDFOREACH()
