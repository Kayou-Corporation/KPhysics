# ----------- KUtils ----------- #
if(NOT TARGET KUtils)
    add_subdirectory(KUtils)
endif()

# ----------- GLM ----------- #
set(DGLM_BUILD_TESTS OFF CACHE BOOL "Disable GLM tests")

add_subdirectory(glm)

# ----------- KTHREADS ----------- #
set(KTHREAD_PERMISSIVE_EXCEPTIONS OFF)
add_subdirectory(KThreads)