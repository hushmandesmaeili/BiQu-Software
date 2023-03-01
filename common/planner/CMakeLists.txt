# include_directories("./")
# include_directories("../../common/include/")
# file(GLOB_RECURSE sources
#         "./*.cpp")

add_library(planner SHARED ${sources})
# target_link_libraries(planner biomimetics)