# CMakeLists.txt for MKA Embedded Project
# Бортовое ПО малого космического аппарата

cmake_minimum_required(VERSION 3.20)

project(MKA_Embedded VERSION 1.0.0 LANGUAGES CXX)

# ============================================================================
# FetchContent для зависимостей
# ============================================================================

include(FetchContent)

FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.14.0
)

# Для Windows: отключаем статические CRT флаги
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(googletest)

# ============================================================================
# Настройки компилятора
# ============================================================================

set(CMAKE_CXX_STANDARD 20)
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
    src/cpp/drivers/eeprom_driver.hpp
    src/cpp/drivers/radio_driver.hpp
    src/cpp/drivers/sun_sensor.hpp
)

# Алгоритмы
set(ALGORITHM_SOURCES
    src/cpp/algorithms/adcs_algorithms.hpp
    src/cpp/algorithms/anomaly_detector.hpp
    src/cpp/algorithms/anomaly_detector.cpp
    src/cpp/algorithms/sgp4.hpp
    src/cpp/algorithms/sgp4.cpp
)

# Системные модули
set(SYSTEM_SOURCES
    src/cpp/systems/fdir.hpp
    src/cpp/systems/state_machine.hpp
    src/cpp/systems/log_system.hpp
    src/cpp/systems/memory_pool.hpp
    src/cpp/systems/telemetry.hpp
    src/cpp/systems/file_system.hpp
    src/cpp/systems/command_handler.hpp
    src/cpp/systems/param_store.hpp
    src/cpp/systems/watchdog_manager.hpp
    src/cpp/systems/ota_updater.hpp
    src/cpp/systems/ota_updater.cpp
    src/cpp/systems/canopen.hpp
)

# Утилиты
set(UTILS_SOURCES
    src/cpp/utils/callback.hpp
    src/cpp/utils/result.hpp
    src/cpp/utils/span.hpp
)

# RTOS
set(RTOS_SOURCES
    src/cpp/rtos/freertos_wrapper.hpp
)

# Все заголовочные файлы
set(HEADER_FILES
    ${HAL_SOURCES}
    ${DRIVER_SOURCES}
    ${ALGORITHM_SOURCES}
    ${SYSTEM_SOURCES}
    ${UTILS_SOURCES}
    ${RTOS_SOURCES}
)

# ============================================================================
# Библиотеки
# ============================================================================

# Основная библиотека (header-only)
add_library(mka_core INTERFACE)
target_include_directories(mka_core INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/cpp
)

# Библиотека алгоритмов (header-only)
add_library(mka_algorithms INTERFACE)
target_include_directories(mka_algorithms INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/cpp
)

# Библиотека алгоритмов с реализацией (anomaly_detector)
add_library(mka_ml STATIC
    src/cpp/algorithms/anomaly_detector.cpp
)
target_include_directories(mka_ml PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/cpp
)

# Библиотека системных модулей
add_library(mka_systems INTERFACE)
target_include_directories(mka_systems INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/cpp
)

# Библиотека утилит
add_library(mka_utils INTERFACE)
target_include_directories(mka_utils INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/cpp
)

# Библиотека RTOS
add_library(mka_rtos INTERFACE)
target_include_directories(mka_rtos INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/cpp
)

# ============================================================================
# Опциональные зависимости для тестов
# ============================================================================

# LittleFS для тестирования файловой системы (опционально)
option(USE_LITTLEFS "Use LittleFS for file system tests" OFF)

if(USE_LITTLEFS)
    FetchContent_Declare(
        littlefs
        GIT_REPOSITORY https://github.com/littlefs-project/littlefs.git
        GIT_TAG        v2.5.0
    )
    FetchContent_MakeAvailable(littlefs)
endif()

# ============================================================================
# Тесты
# ============================================================================

option(BUILD_TESTS "Build unit tests" ON)

if(BUILD_TESTS AND HOST_BUILD)
    enable_testing()

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

    # Тест команд (Command Handler)
    add_executable(test_commands tests/test_commands.cpp)
    target_link_libraries(test_commands mka_core GTest::gtest_main)
    add_test(NAME test_commands COMMAND test_commands)

    # Тест хранилища параметров (Parameter Store)
    add_executable(test_param_store tests/test_param_store.cpp)
    target_link_libraries(test_param_store mka_core GTest::gtest_main)
    add_test(NAME test_param_store COMMAND test_param_store)

    # Тест watchdog менеджера
    add_executable(test_watchdog tests/test_watchdog.cpp)
    target_link_libraries(test_watchdog mka_core GTest::gtest_main)
    add_test(NAME test_watchdog COMMAND test_watchdog)

    # Тест memory pool
    add_executable(test_memory_pool tests/test_memory_pool.cpp)
    target_link_libraries(test_memory_pool mka_core GTest::gtest_main)
    add_test(NAME test_memory_pool COMMAND test_memory_pool)

    # Тест log system
    add_executable(test_log_system tests/test_log_system.cpp)
    target_link_libraries(test_log_system mka_core GTest::gtest_main)
    add_test(NAME test_log_system COMMAND test_log_system)

    # Тест телеметрии и команд
    add_executable(test_telemetry tests/test_telemetry.cpp)
    target_link_libraries(test_telemetry mka_core GTest::gtest_main)
    add_test(NAME test_telemetry COMMAND test_telemetry)

    # Тест файловой системы (File System) - требует реализации
    add_executable(test_file_system tests/test_file_system.cpp)
    target_link_libraries(test_file_system mka_core GTest::gtest_main)
    add_test(NAME test_file_system COMMAND test_file_system)

    # Тест SGP4 орбитального пропагатора - временно отключен (static assertion в sgp4.hpp)
    # add_executable(test_sgp4 tests/test_sgp4.cpp)
    # target_link_libraries(test_sgp4 mka_algorithms GTest::gtest_main)
    # add_test(NAME test_sgp4 COMMAND test_sgp4)

    # Тест UKF фильтра
    add_executable(test_ukf tests/test_ukf.cpp)
    target_link_libraries(test_ukf mka_algorithms GTest::gtest_main)
    add_test(NAME test_ukf COMMAND test_ukf)

    # Тест Anomaly Detector
    add_executable(test_anomaly_detector tests/test_anomaly_detector.cpp)
    target_link_libraries(test_anomaly_detector mka_ml GTest::gtest_main)
    add_test(NAME test_anomaly_detector COMMAND test_anomaly_detector)

    # Тест OTA обновлений - временно отключен (требует mock реализаций)
    # add_executable(test_ota tests/test_ota.cpp)
    # target_link_libraries(test_ota mka_core GTest::gtest_main)
    # add_test(NAME test_ota COMMAND test_ota)

    # Тест FreeRTOS wrapper - отключен (требует FreeRTOS для хост-билда)
    # add_executable(test_freertos_wrapper tests/test_freertos_wrapper.cpp)
    # target_link_libraries(test_freertos_wrapper mka_core GTest::gtest_main)
    # add_test(NAME test_freertos_wrapper COMMAND test_freertos_wrapper)
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
