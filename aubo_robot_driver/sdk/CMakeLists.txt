

message("data:${CMAKE_INSTALL_PREFIX}")
install(DIRECTORY sdk 
  DESTINATION .)
include_directories(include ${CMAKE_INSTALL_PREFIX}/sdk/inc)
# link_directories(${CMAKE_INSTALL_PREFIX}/sdk/lib64)

set(auborobotcontroller_LIBRARY ${CMAKE_INSTALL_PREFIX}/sdk/lib64/libauborobotcontroller.so.1)
set(log4cplus_LIBRARY ${CMAKE_INSTALL_PREFIX}/sdk/lib64/liblog4cplus-1.2.so.5)
# install(TARGETS auborobotcontroller
#   DESTINATION lib/${PROJECT_NAME} 
# )
