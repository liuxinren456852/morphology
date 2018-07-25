cmake_minimum_required(VERSION 3.3)
project(morphology)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(dir  "D:/Mapping/Tools/morphology/Debug")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${dir})

#[[#include_directories(${MAPPING_INCLUDE_DIR}/Photogrammetry/include)
include_directories(D:/MappingLibrary/Mapping/Library/Photogrammetry/include)
include_directories(D:/MappingLibrary/Mapping/Library/Buildings/include)
include_directories(D:/MappingLibrary/Mapping/Library/LaserScan/include)
include_directories(D:/MappingLibrary/Mapping/Foreign/ANN/include/ANN)
include_directories(D:/MappingLibrary/Mapping/Foreign/ANN/src)
#link_directories(D:/MappingLibrary/Mapping/Library/lib64)
link_directories(${MAPPING_LIB_DIR})
link_libraries(-lLaserScan -lPhotogrammetry -lBuildings -lEispack -lLinpack -lANN -lnewmat -lgfortran -g3 -llas)]]

set (MAPPING_LIB_DIR "D:/Mapping/Library/lib64") #D:\MappingLibrary\Mapping\Library\lib64
set (MAPPING_INCLUDE_DIR "D:/Mapping/Library")

#include_directories(${MAPPING_INCLUDE_DIR}/Photogrammetry/include)
include_directories(D:/Mapping/Library/Photogrammetry/include)
include_directories(D:/Mapping/Library/Buildings/include)
include_directories(D:/Mapping/Library/LaserScan/include)
include_directories(D:/Mapping/Foreign/ANN/include/ANN)
include_directories(D:/Mapping/Foreign/ANN/src)
#link_directories(D:/Mapping/Library/lib64)
link_directories(${MAPPING_LIB_DIR})
link_libraries(-lLaserScan -lPhotogrammetry -lBuildings -lEispack -lLinpack -lANN -lnewmat -lgfortran -g3 -llas)


set(SOURCE_FILES main.cpp Laservoxel.cpp morphology_indoor.cpp segmentation.cpp 3dbbox.cpp
        ConnCompSegmentation.cpp ConnCompSegmentation.h)
add_executable(morphology ${SOURCE_FILES})