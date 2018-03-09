cmake_minimum_required(VERSION 2.8.3)
project(vega_fem)

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    corotationalLinearFEM
    corotationalLinearFEMForceModel
    volumetricMeshLoader
    generateMassMatrix
    implicitBackwardEulerSparse
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

### LIBRARIES
add_library(corotationalLinearFEM
  src/corotationalLinearFEM.cpp
  src/utils/eig3.cpp
  src/utils/mat3d.cpp
  src/utils/polarDecomposition.cpp
  src/utils/sparseMatrix.cpp
  src/utils/vec3d.cpp
  src/utils/volumetricMeshENuMaterial.cpp
  src/utils/volumetricMeshOrthotropicMaterial.cpp
)
target_link_libraries(corotationalLinearFEM
  ${catkin_LIBRARIES}
)

add_library(corotationalLinearFEMForceModel
  src/corotationalLinearFEMForceModel.cpp
  src/utils/forceModel.cpp
)
target_link_libraries(corotationalLinearFEMForceModel
  ${catkin_LIBRARIES}
)

add_library(generateMassMatrix
  src/generateMassMatrix.cpp

)
target_link_libraries(generateMassMatrix
  ${catkin_LIBRARIES}
)

add_library(implicitBackwardEulerSparse
  src/implicitBackwardEulerSparse.cpp
  src/utils/CGSolver.cpp
  src/utils/implicitNewmarkSparse.cpp
  src/utils/insertRows.cpp
  src/utils/integratorBase.cpp
  src/utils/integratorBaseSparse.cpp
  src/utils/linearSolver.cpp
)
target_link_libraries(implicitBackwardEulerSparse
  ${catkin_LIBRARIES}
)

add_library(volumetricMeshLoader
  src/volumetricMeshLoader.cpp
  src/utils/cubicMesh.cpp
  src/utils/tetMesh.cpp
  src/utils/volumetricMesh.cpp
  src/utils/volumetricMeshENuMaterial.cpp
  src/utils/volumetricMeshMooneyRivlinMaterial.cpp
  src/utils/volumetricMeshOrthotropicMaterial.cpp
  src/utils/volumetricMeshParser.cpp

)
target_link_libraries(volumetricMeshLoader
  ${catkin_LIBRARIES}
)

## INSTALLS
install(
  TARGETS
    corotationalLinearFEM
    corotationalLinearFEMForceModel
    generateMassMatrix
    implicitBackwardEulerSparse
    volumetricMeshLoader
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
