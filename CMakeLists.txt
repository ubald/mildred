cmake_minimum_required(VERSION 3.5)

# This is only for CLion, do not use for build

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
    add_subdirectory(src/${subdir} ${subdir})
endforeach ()
