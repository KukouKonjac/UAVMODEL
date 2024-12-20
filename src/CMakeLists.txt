file(GLOB_RECURSE CURRENT_PROJ_SRC ${CMAKE_CURRENT_SOURCE_DIR}/model/**.cpp)
set(MODEL_SRC ${CURRENT_PROJ_SRC} PARENT_SCOPE)# Set the source file list to the parent scope£¬so that it can be used by test folder
set(MODEL_SRC ${CURRENT_PROJ_SRC})
# message(STATUS "src file:" ${MODEL_SRC})

add_subdirectory(platform)
# Ìí¼Ó×ÓÄ¿Â¼
add_subdirectory(model/hull/quadrotor)