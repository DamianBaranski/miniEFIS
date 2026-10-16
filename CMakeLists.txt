# MiniEFIS: flight instruments for the Waveshare ESP32-S3 480x480 RGB panel.
#
#   mkdir -p build && cd build
#   cmake ..
#   make                # firmware
#   make flash          # flash it (ESPPORT=/dev/ttyUSB0 to pick a port)
#   make monitor        # serial console
#   make host-tests     # build and run the attitude filter tests on this PC
#
# ESP-IDF is normally driven through idf.py, which is a wrapper that
# installs nothing but does set up the cross toolchain, the target and the
# Python environment before calling cmake. This file does that setup
# itself, so a plain "cmake .." is enough: it finds ESP-IDF, downloads and
# installs it if it is missing, exports the same environment
# export.sh would, and only then hands over to ESP-IDF's own build system.
#
# The first configure therefore takes a while and pulls down a few GB.
# Pass -DESP_IDF_AUTO_INSTALL=OFF to require an existing installation
# instead, or -DIDF_PATH=/path/to/esp-idf to point at one.
cmake_minimum_required(VERSION 3.16)

set(ESP_IDF_VERSION_TAG "v5.4.4" CACHE STRING
    "ESP-IDF release to install when none is found (v5.2 or newer)")
set(ESP_IDF_INSTALL_DIR "$ENV{HOME}/esp/esp-idf" CACHE PATH
    "Where to install ESP-IDF if it has to be downloaded")
option(ESP_IDF_AUTO_INSTALL
    "Download and install ESP-IDF automatically when it is missing" ON)
set(IDF_TARGET "esp32s3" CACHE STRING "Chip to build for")

# ---------------------------------------------------------------------------
# 1. Find an existing ESP-IDF
# ---------------------------------------------------------------------------

function(find_esp_idf out_var)
    foreach(candidate
            "${IDF_PATH}"
            "$ENV{IDF_PATH}"
            "${ESP_IDF_INSTALL_DIR}"
            "$ENV{HOME}/esp/esp-idf"
            "/opt/esp-idf")
        if(candidate AND EXISTS "${candidate}/tools/cmake/project.cmake")
            set(${out_var} "${candidate}" PARENT_SCOPE)
            return()
        endif()
    endforeach()

    set(${out_var} "" PARENT_SCOPE)
endfunction()

find_esp_idf(IDF_ROOT)

# ---------------------------------------------------------------------------
# 2. Install it if there is none
# ---------------------------------------------------------------------------

if(NOT IDF_ROOT)
    if(NOT ESP_IDF_AUTO_INSTALL)
        message(FATAL_ERROR
            "ESP-IDF was not found and ESP_IDF_AUTO_INSTALL is OFF.\n"
            "Point at an installation with -DIDF_PATH=/path/to/esp-idf, "
            "or re-run without ESP_IDF_AUTO_INSTALL=OFF to have it "
            "installed into ${ESP_IDF_INSTALL_DIR}.\n")
    endif()

    find_package(Git REQUIRED)

    message(STATUS "")
    message(STATUS "ESP-IDF not found. Installing ${ESP_IDF_VERSION_TAG} "
                   "into ${ESP_IDF_INSTALL_DIR}")
    message(STATUS "This downloads a few GB and takes several minutes; it "
                   "only happens once.")
    message(STATUS "")

    get_filename_component(idf_parent "${ESP_IDF_INSTALL_DIR}" DIRECTORY)
    file(MAKE_DIRECTORY "${idf_parent}")

    # Shallow, so this is a download of the pinned release rather than of
    # the project's whole history.
    execute_process(
        COMMAND "${GIT_EXECUTABLE}" clone
                --branch "${ESP_IDF_VERSION_TAG}"
                --depth 1 --recursive --shallow-submodules
                https://github.com/espressif/esp-idf.git
                "${ESP_IDF_INSTALL_DIR}"
        RESULT_VARIABLE clone_result)

    if(NOT clone_result EQUAL 0)
        message(FATAL_ERROR
            "Cloning ESP-IDF failed (git exited ${clone_result}).\n"
            "Check the network connection, then delete "
            "${ESP_IDF_INSTALL_DIR} and configure again.\n")
    endif()

    set(IDF_ROOT "${ESP_IDF_INSTALL_DIR}")
endif()

message(STATUS "Using ESP-IDF at ${IDF_ROOT}")
set(ENV{IDF_PATH} "${IDF_ROOT}")

# An interrupted clone leaves project.cmake in place but the submodules
# empty, which looks like a working installation and then fails much
# later with missing components. Check and repair instead.
find_package(Git QUIET)

if(GIT_FOUND AND EXISTS "${IDF_ROOT}/.git")
    execute_process(
        COMMAND "${GIT_EXECUTABLE}" -C "${IDF_ROOT}" submodule status
        OUTPUT_VARIABLE submodule_status
        RESULT_VARIABLE submodule_result
        ERROR_QUIET)

    # git marks an uninitialised submodule with a leading "-".
    if(submodule_result EQUAL 0 AND submodule_status MATCHES "(^|\n)-")
        message(STATUS "Fetching missing ESP-IDF submodules")

        execute_process(
            COMMAND "${GIT_EXECUTABLE}" -C "${IDF_ROOT}"
                    submodule update --init --recursive --depth 1
            RESULT_VARIABLE submodule_update_result)

        if(NOT submodule_update_result EQUAL 0)
            message(FATAL_ERROR
                "Could not fetch the ESP-IDF submodules "
                "(git exited ${submodule_update_result}).\n"
                "Finish the checkout by hand:\n"
                "  git -C ${IDF_ROOT} submodule update --init --recursive\n")
        endif()
    endif()
endif()

# ---------------------------------------------------------------------------
# 3. Set up the environment export.sh would have
# ---------------------------------------------------------------------------

# Rather than reimplementing what export.sh does - and picking the wrong
# Python while doing it, since the first python3 on PATH is often a conda
# or pyenv one that ESP-IDF does not support - source the real thing in a
# subshell and adopt the variables it sets.
set(export_script "${CMAKE_BINARY_DIR}/idf-export-env.sh")

file(WRITE "${export_script}"
"#!/usr/bin/env bash\n"
"# Generated by CMakeLists.txt. Sources ESP-IDF's own export.sh and\n"
"# prints the environment the build needs, so cmake can adopt it.\n"
"export IDF_PATH='${IDF_ROOT}'\n"
". \"$IDF_PATH/export.sh\" > /dev/null 2>&1 || exit 1\n"
"printf 'PATH=%s\\n' \"$PATH\"\n"
"printf 'IDF_PATH=%s\\n' \"$IDF_PATH\"\n"
"printf 'IDF_PYTHON_ENV_PATH=%s\\n' \"$IDF_PYTHON_ENV_PATH\"\n"
"printf 'OPENOCD_SCRIPTS=%s\\n' \"$OPENOCD_SCRIPTS\"\n"
"printf 'ESP_ROM_ELF_DIR=%s\\n' \"$ESP_ROM_ELF_DIR\"\n"
"printf 'IDF_PYTHON=%s\\n' \"$(command -v python)\"\n")

execute_process(
    COMMAND bash "${export_script}"
    OUTPUT_VARIABLE idf_env
    RESULT_VARIABLE export_result
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET)

# export.sh fails when the toolchain has not been downloaded yet, which is
# the state a fresh checkout is in.
if(NOT export_result EQUAL 0)
    message(STATUS "")
    message(STATUS "Installing the ESP-IDF tools for ${IDF_TARGET}")
    message(STATUS "")

    execute_process(
        COMMAND bash "${IDF_ROOT}/install.sh" "${IDF_TARGET}"
        WORKING_DIRECTORY "${IDF_ROOT}"
        RESULT_VARIABLE install_result)

    if(NOT install_result EQUAL 0)
        message(FATAL_ERROR
            "ESP-IDF's install.sh failed (exit ${install_result}).\n"
            "The usual cause is the python3 first on PATH being a version "
            "ESP-IDF does not support - a conda or pyenv interpreter, for "
            "instance. Run it by hand to see the actual complaint:\n"
            "  ${IDF_ROOT}/install.sh ${IDF_TARGET}\n")
    endif()

    execute_process(
        COMMAND bash "${export_script}"
        OUTPUT_VARIABLE idf_env
        RESULT_VARIABLE export_result
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET)

    if(NOT export_result EQUAL 0)
        message(FATAL_ERROR
            "ESP-IDF is installed but its environment could not be read.\n"
            "Try it by hand:\n"
            "  . ${IDF_ROOT}/export.sh\n"
            "and if that works, configure again from the same shell.\n")
    endif()
endif()

string(REPLACE "\n" ";" idf_env_lines "${idf_env}")

foreach(line IN LISTS idf_env_lines)
    if(line MATCHES "^([A-Za-z_][A-Za-z0-9_]*)=(.*)$")
        set(key "${CMAKE_MATCH_1}")
        set(value "${CMAKE_MATCH_2}")
        set(ENV{${key}} "${value}")
    endif()
endforeach()

if(NOT EXISTS "$ENV{IDF_PYTHON_ENV_PATH}")
    message(FATAL_ERROR
        "ESP-IDF's Python environment is missing "
        "(IDF_PYTHON_ENV_PATH='$ENV{IDF_PYTHON_ENV_PATH}').\n"
        "Run ${IDF_ROOT}/install.sh ${IDF_TARGET} and configure again.\n")
endif()

# Pin the interpreter by absolute path, as idf.py does.
#
# Left to itself ESP-IDF defaults the PYTHON build property to the bare
# name "python" (tools/cmake/build.cmake), resolved from PATH at the
# moment each command runs. Configure-time commands would find the right
# one, because the environment adopted above applies to this process -
# but build-time ones would not: make and the nested bootloader build
# inherit the environment of whichever shell invoked them, where ESP-IDF
# has not been exported. The bootloader then fails to find
# idf_component_manager. An absolute path is immune to that, and ESP-IDF
# forwards it to the bootloader sub-project for us.
if(IDF_PYTHON AND EXISTS "${IDF_PYTHON}")
    set(idf_python "${IDF_PYTHON}")
elseif(EXISTS "$ENV{IDF_PYTHON_ENV_PATH}/bin/python")
    set(idf_python "$ENV{IDF_PYTHON_ENV_PATH}/bin/python")
else()
    message(FATAL_ERROR
        "Could not find ESP-IDF's Python interpreter in "
        "'$ENV{IDF_PYTHON_ENV_PATH}'.\n"
        "Run ${IDF_ROOT}/install.sh ${IDF_TARGET} and configure again.\n")
endif()

message(STATUS "Using ESP-IDF Python at ${idf_python}")
set(PYTHON "${idf_python}" CACHE FILEPATH
    "Python interpreter ESP-IDF builds with" FORCE)

# A shell snippet with the same environment, for the few tools that are
# looked up by name rather than by path - openocd and gdb, if you attach a
# debugger. Not needed to build, flash or monitor.
file(WRITE "${CMAKE_BINARY_DIR}/idf-env.sh"
"# Generated. Source this for an interactive ESP-IDF shell:\n"
"#   . ${CMAKE_BINARY_DIR}/idf-env.sh\n"
". \"${IDF_ROOT}/export.sh\"\n")

# Carry the environment into build time as well.
#
# Everything above applies to this cmake process, which is enough for
# configure-time commands - but not for build-time ones. make, and the
# nested cmake that configures the bootloader, inherit the environment of
# whatever shell ran them, where ESP-IDF has not been exported. The
# bootloader's toolchain file names its compilers bare
# ("xtensa-esp32s3-elf-gcc") and resolves them from PATH at that point,
# so it fails exactly like it would in a fresh terminal.
#
# Prefixing every custom command with an environment wrapper fixes the
# whole class of problem in one place, rather than forwarding variables
# one at a time as each is discovered. Compile and link rules are left
# alone: those already hold absolute paths from this project's cache, and
# an extra process per source file is not worth it.
set_property(GLOBAL PROPERTY RULE_LAUNCH_CUSTOM
    "\"${CMAKE_COMMAND}\" -E env \"PATH=$ENV{PATH}\" \"IDF_PATH=$ENV{IDF_PATH}\" \"IDF_PYTHON_ENV_PATH=$ENV{IDF_PYTHON_ENV_PATH}\"")

# ---------------------------------------------------------------------------
# 4. Hand over to ESP-IDF
# ---------------------------------------------------------------------------

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(miniefis)

# ---------------------------------------------------------------------------
# 5. The host tests, as a nested build with this machine's compiler
# ---------------------------------------------------------------------------

# Not part of "all": the firmware build must not depend on the host
# toolchain, and the tests are a separate cmake project for that reason.
add_custom_target(host-tests
    COMMAND "${CMAKE_COMMAND}" -S "${CMAKE_SOURCE_DIR}/test"
                               -B "${CMAKE_BINARY_DIR}/host-tests"
    COMMAND "${CMAKE_COMMAND}" --build "${CMAKE_BINARY_DIR}/host-tests"
    COMMAND "${CMAKE_CTEST_COMMAND}" --test-dir "${CMAKE_BINARY_DIR}/host-tests"
                                     --output-on-failure
    COMMENT "Building and running the attitude filter tests on this PC"
    VERBATIM)
