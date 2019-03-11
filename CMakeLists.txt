cmake_minimum_required(VERSION 3.5)
#project(mildred)

#add_subdirectory(src/mildred_core mildred_core EXCLUDE_FROM_ALL)
#add_subdirectory(src/mildred_dynamixel mildred_dynamixel EXCLUDE_FROM_ALL)

macro(subdirlist result curdir)
    file(GLOB children RELATIVE ${curdir} ${curdir}/*)
    set(dirlist "")

    message("${curdir}")
    foreach (child ${children})
        if (IS_DIRECTORY ${curdir}/${child})
            list(APPEND dirlist ${child})
        endif ()
    endforeach ()
    set(${result} ${dirlist})
endmacro()

subdirlist(SUBDIRS ${CMAKE_CURRENT_SOURCE_DIR}/src)
foreach (subdir ${SUBDIRS})
    message(${subdir})
    add_subdirectory(src/${subdir} ${subdir})
endforeach ()
