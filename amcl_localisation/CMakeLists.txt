project(amcl_localisation)
cmake_minimum_required(VERSION 3.25)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}/maps
)
