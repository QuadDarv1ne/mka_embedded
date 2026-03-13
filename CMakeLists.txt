# CMakeLists.txt for MKA Embedded Project
# Бортовое ПО малого космического аппарата

cmake_minimum_required(VERSION 3.20)

project(MKA_Embedded VERSION 1.0.0 LANGUAGES CXX)

# ============================================================================
# Настройки компилятора
# ============================================================================

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Оптимизация и предупреждения
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(
        -Wall
        -Wextra
        -Wpedantic
        -Werror=return-type
        -Werror=unused-result
        -ffunction-sections
        -fdata-sections
    )
    
    if(CMAKE_BUILD_TYPE STREQUAL "Release")
        add_compile_options(-Os -flto)
    endif()
endif()

# ============================================================================
# Целевая платформа
# ============================================================================

option(STM32F4 "Build for STM32F4" OFF)
option(STM32F7 "Build for STM32F7" OFF)
option(STM32H7 "Build for STM32H7" OFF)
option(HOST_BUILD "Build for host testing" ON)

# ============================================================================
# Определения препроцессора
# ============================================================================

add_compile_definitions(
    MKA_VERSION_MAJOR=1
    MKA_VERSION_MINOR=0
    MKA_VERSION_PATCH=0
)

if(STM32F4)
    add_compile_definitions(
        STM32F4
        ARM_MATH_CM4
        __FPU_PRESENT=1
    )
    set(TARGET_MCU "stm32f4")
elseif(STM32F7)
    add_compile_definitions(
        STM32F7
        ARM_MATH_CM7
        __FPU_PRESENT=1
    )
    set(TARGET_MCU "stm32f7")
elseif(STM32H7)
    add_compile_definitions(
        STM32H7
        ARM_MATH_CM7
        __FPU_PRESENT=1
    )
    set(TARGET_MCU "stm32h7")
else()
    set(TARGET_MCU "host")
endif()

# ============================================================================
# Исходные файлы
# ============================================================================

# HAL интерфейсы
set(HAL_SOURCES
    src/cpp/hal/hal_full.hpp
)

# Драйверы
set(DRIVER_SOURCES
    src/cpp/drivers/sensors_drivers.hpp
)

# Алгоритмы
set(ALGORITHM_SOURCES
    src/cpp/algorithms/adcs_algorithms.hpp
)

# Системные модули
set(SYSTEM_SOURCES
    src/cpp/systems/fdir.hpp
    src/cpp/systems/state_machine.hpp
    src/cpp/systems/log_system.hpp
    src/cpp/systems/memory_pool.hpp
)

# Все заголовочные файлы
set(HEADER_FILES
    ${HAL_SOURCES}
    ${DRIVER_SOURCES}
    ${ALGORITHM_SOURCES}
    ${SYSTEM_SOURCES}
)

# ============================================================================
# Библиотеки
# ============================================================================

# Основная библиотека (header-only)
add_library(mka_core INTERFACE)
target_include_directories(mka_core INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/cpp
)

# Библиотека алгоритмов
add_library(mka_algorithms INTERFACE)
target_include_directories(mka_algorithms INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/cpp
)

# ============================================================================
# Тесты
# ============================================================================

option(BUILD_TESTS "Build unit tests" ON)

if(BUILD_TESTS AND HOST_BUILD)
    enable_testing()
    
    # Google Test
    find_package(GTest QUIET)
    
    if(GTest_FOUND)
        # Тест алгоритмов
        add_executable(test_algorithms tests/test_algorithms.cpp)
        target_link_libraries(test_algorithms mka_algorithms GTest::gtest_main)
        add_test(NAME test_algorithms COMMAND test_algorithms)

        # Тест FDIR
        add_executable(test_fdir tests/test_fdir.cpp)
        target_link_libraries(test_fdir mka_core GTest::gtest_main)
        add_test(NAME test_fdir COMMAND test_fdir)

        # Тест State Machine
        add_executable(test_statemachine tests/test_statemachine.cpp)
        target_link_libraries(test_statemachine mka_core GTest::gtest_main)
        add_test(NAME test_statemachine COMMAND test_statemachine)

        # Тест утилит (Result, Callback)
        add_executable(test_utils tests/test_utils.cpp)
        target_link_libraries(test_utils mka_core GTest::gtest_main)
        add_test(NAME test_utils COMMAND test_utils)

    else()
        message(STATUS "Google Test not found, skipping unit tests")
    endif()
endif()

# ============================================================================
# Документация
# ============================================================================

option(BUILD_DOCS "Build documentation" OFF)

if(BUILD_DOCS)
    find_package(Doxygen QUIET)
    
    if(Doxygen_FOUND)
        set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile.in)
        set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
        
        configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
        
        add_custom_target(docs
            COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Generating API documentation with Doxygen"
            VERBATIM
        )
    endif()
endif()

# ============================================================================
# Статический анализ
# ============================================================================

option(ENABLE_CPPCHECK "Enable static analysis with cppcheck" OFF)
option(ENABLE_CLANG_TIDY "Enable static analysis with clang-tidy" OFF)

# Cppcheck настройки
if(ENABLE_CPPCHECK)
    find_program(CPPCHECK cppcheck)
    if(CPPCHECK)
        set(CMAKE_CXX_CPPCHECK
            ${CPPCHECK}
            --enable=all
            --error-exitcode=1
            --inline-suppr
            --std=c++17
            --suppress=missingIncludeSystem
            --suppress=unusedFunction
            --force
            -j4
        )
        message(STATUS "Cppcheck enabled: ${CPPCHECK}")
    else()
        message(WARNING "Cppcheck requested but not found")
    endif()
endif()

# Clang-tidy настройки
if(ENABLE_CLANG_TIDY)
    find_program(CLANGTIDY clang-tidy)
    if(CLANGTIDY)
        set(CMAKE_CXX_CLANG_TIDY
            ${CLANGTIDY}
            --format-style=file
            --header-filter=src/.*
            --checks=
                -bugprone-*,
                -cert-*,
                -clang-analyzer-*,
                -cppcoreguidelines-*,
                -misc-*,
                -performance-*,
                -portability-*,
                -readability-*,
                -use-trailing-return-type,
                -readability-magic-numbers,
                -cppcoreguidelines-pro-bounds-pointer-arithmetic,
                -cppcoreguidelines-avoid-magic-numbers,
                -readability-identifier-length
        )
        message(STATUS "Clang-tidy enabled: ${CLANGTIDY}")
    else()
        message(WARNING "Clang-tidy requested but not found")
    endif()
endif()

# Code coverage настройки
option(ENABLE_COVERAGE "Enable code coverage with gcov" OFF)

if(ENABLE_COVERAGE AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(--coverage -fprofile-arcs -ftest-coverage -O0)
    add_link_options(--coverage)
    message(STATUS "Code coverage enabled")
endif()

# ============================================================================
# Размер бинарника
# ============================================================================

if(NOT HOST_BUILD)
    # Скрипт для вывода размера секций
    add_custom_command(TARGET mka_core POST_BUILD
        COMMAND ${CMAKE_SIZE_UTIL} $<TARGET_FILE:mka_core>
        COMMENT "Memory usage:"
    )
endif()

# ============================================================================
# Информация о сборке
# ============================================================================

message(STATUS "")
message(STATUS "MKA Embedded Build Configuration")
message(STATUS "=================================")
message(STATUS "Version:     ${PROJECT_VERSION}")
message(STATUS "Target:      ${TARGET_MCU}")
message(STATUS "Build type:  ${CMAKE_BUILD_TYPE}")
message(STATUS "Compiler:    ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Tests:       ${BUILD_TESTS}")
message(STATUS "Docs:        ${BUILD_DOCS}")
message(STATUS "")
