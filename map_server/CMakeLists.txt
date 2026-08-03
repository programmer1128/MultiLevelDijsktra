cmake_minimum_required(VERSION 3.5)
project(map_server CXX)

# Force C++20 for Coroutines
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(CheckIncludeFileCXX)

# Find External Dependencies
find_package(Drogon CONFIG REQUIRED)
find_package(ZLIB REQUIRED)

# Recursively collect all source files (.cpp, .cc, .c)
file(GLOB DATA_SRC CONFIGURE_DEPENDS "load_data/*.cc" "load_data/*.cpp")
file(GLOB SEARCH_SRC CONFIGURE_DEPENDS "search/*.cc" "search/*.cpp")

# ==============================================================================
# 1. CREATE THE EXECUTABLE TARGET FIRST
# ==============================================================================
add_executable(${PROJECT_NAME} main.cc)


drogon_create_views(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/views
                    ${CMAKE_CURRENT_BINARY_DIR})

target_sources(${PROJECT_NAME}
               PRIVATE
               ${CTL_SRC}
               ${FILTER_SRC}
               ${PLUGIN_SRC}
               ${MODEL_SRC}
               ${DATA_SRC}
               ${SEARCH_SRC})

target_include_directories(${PROJECT_NAME}
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
                                   ${CMAKE_CURRENT_SOURCE_DIR}/models)

# ==============================================================================
# 3. LINK LIBRARIES TO THE TARGET
# ==============================================================================
target_link_libraries(${PROJECT_NAME} PRIVATE 
    Drogon::Drogon 
    ZLIB::ZLIB
)

if (CMAKE_CXX_STANDARD LESS 20)
    message(FATAL_ERROR "C++20 is required for coroutines!")
else ()
    message(STATUS "Using C++20 standard with Coroutines enabled.")
endif ()

# Optional test directory
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/CMakeLists.txt")
    add_subdirectory(test)
endif()