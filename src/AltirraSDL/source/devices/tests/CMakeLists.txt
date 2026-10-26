# Embedded FujiNet-NIO SIO adapter regression.  This compiles the production
# custom device and drives its real IATDeviceSIO callbacks with a recording
# SIO manager; it deliberately does not model Q/W/P/R itself.
add_executable(altirra-fujinet-nio-sio-test
    ../fujinetnio.cpp
    ${ALTIRRA_FUJINET_NIO_SOURCE_DIR}/src/platform/posix/time.cpp
    fujinetnio_sio_test.cpp)

target_include_directories(altirra-fujinet-nio-sio-test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../stubs
    ${PROJECT_SOURCE_DIR}/src/Altirra/h
    ${PROJECT_SOURCE_DIR}/src/Altirra/source
    ${PROJECT_SOURCE_DIR}/src/AltirraSDL/source
    ${PROJECT_SOURCE_DIR}/src/compat
    ${PROJECT_SOURCE_DIR}/src/Altirra/autobuild_default)

target_compile_definitions(altirra-fujinet-nio-sio-test PRIVATE
    ALTIRRA_FUJINET_NIO=1
    ALTIRRA_FUJINET_NIO_TEST_CLOCK=1
    FN_POSIX_TEST_CLOCK=1)
target_link_libraries(altirra-fujinet-nio-sio-test PRIVATE
    fujinet-nio ATCPU ATEmulation ATDevices ATIO ATAudio ATNetwork
    ATNetworkSockets ATDebugger ATVM ATCore Kasumi vdjson system SDL3::SDL3)

add_test(NAME altirra-fujinet-nio-sio
    COMMAND altirra-fujinet-nio-sio-test
            ${ALTIRRA_FUJINET_NIO_SOURCE_DIR}/tests/fixtures/atari-sio/nio_v1.json)
set_tests_properties(altirra-fujinet-nio-sio PROPERTIES TIMEOUT 60)
