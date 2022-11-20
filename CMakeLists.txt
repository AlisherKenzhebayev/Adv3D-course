cmake_minimum_required(VERSION 3.10)

project(NPGR010)

add_executable(NPGR010 src/pg3render.cpp)

target_sources(NPGR010 PRIVATE 
  src/camera.hpp
  src/config.hpp
  src/framebuffer.hpp
  src/geometry.hpp
  src/lights.hpp
  src/materials.hpp
  src/math.hpp
  src/pathtracer.hpp
  src/ray.hpp
  src/renderer.hpp
  src/rng.hpp
  src/scene.hpp
  src/utils.hpp
)

target_compile_features(NPGR010 PRIVATE cxx_std_17)

find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    target_link_libraries(NPGR010 PUBLIC OpenMP::OpenMP_CXX)
endif()