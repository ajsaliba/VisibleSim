# Install script for directory: C:/Users/anton/VisibleSim

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Program Files (x86)/VisibleSim")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set path to fallback-tool for dependency-resolution.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "C:/msys64/mingw64/bin/objdump.exe")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/../applicationsBin/stressTestSB" TYPE EXECUTABLE FILES "C:/Users/anton/VisibleSim/build-mingw64/stressTestSB.exe")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/../applicationsBin/stressTestSB/stressTestSB.exe" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/../applicationsBin/stressTestSB/stressTestSB.exe")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "C:/msys64/mingw64/bin/strip.exe" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/../applicationsBin/stressTestSB/stressTestSB.exe")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/applicationsBin/simpleColorBB" TYPE EXECUTABLE FILES "C:/Users/anton/VisibleSim/build-mingw64/simpleColorBB.exe")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/simpleColorBB/simpleColorBB.exe" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/simpleColorBB/simpleColorBB.exe")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "C:/msys64/mingw64/bin/strip.exe" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/simpleColorBB/simpleColorBB.exe")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/applicationsBin/simpleColorSB" TYPE EXECUTABLE FILES "C:/Users/anton/VisibleSim/build-mingw64/simpleColorSB.exe")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/simpleColorSB/simpleColorSB.exe" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/simpleColorSB/simpleColorSB.exe")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "C:/msys64/mingw64/bin/strip.exe" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/simpleColorSB/simpleColorSB.exe")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/applicationsBin/rainbow" TYPE EXECUTABLE FILES "C:/Users/anton/VisibleSim/build-mingw64/ants.exe")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/rainbow/ants.exe" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/rainbow/ants.exe")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "C:/msys64/mingw64/bin/strip.exe" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/rainbow/ants.exe")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/applicationsBin/demoMotionsC3D" TYPE EXECUTABLE FILES "C:/Users/anton/VisibleSim/build-mingw64/demoMotionsC3D.exe")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/demoMotionsC3D/demoMotionsC3D.exe" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/demoMotionsC3D/demoMotionsC3D.exe")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "C:/msys64/mingw64/bin/strip.exe" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/demoMotionsC3D/demoMotionsC3D.exe")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/applicationsBin/demoReconfC3D" TYPE EXECUTABLE FILES "C:/Users/anton/VisibleSim/build-mingw64/demoReconfC3D.exe")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/demoReconfC3D/demoReconfC3D.exe" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/demoReconfC3D/demoReconfC3D.exe")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "C:/msys64/mingw64/bin/strip.exe" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/demoReconfC3D/demoReconfC3D.exe")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/applicationsBin/myMotionTest" TYPE EXECUTABLE FILES "C:/Users/anton/VisibleSim/build-mingw64/myMotionTest.exe")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/myMotionTest/myMotionTest.exe" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/myMotionTest/myMotionTest.exe")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "C:/msys64/mingw64/bin/strip.exe" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/myMotionTest/myMotionTest.exe")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/applicationsBin/hexanodes_demo" TYPE EXECUTABLE FILES "C:/Users/anton/VisibleSim/build-mingw64/hexanodes_demo.exe")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/hexanodes_demo/hexanodes_demo.exe" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/hexanodes_demo/hexanodes_demo.exe")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "C:/msys64/mingw64/bin/strip.exe" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/hexanodes_demo/hexanodes_demo.exe")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/applicationsBin/comeBack" TYPE EXECUTABLE FILES "C:/Users/anton/VisibleSim/build-mingw64/comeBack.exe")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/comeBack/comeBack.exe" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/comeBack/comeBack.exe")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "C:/msys64/mingw64/bin/strip.exe" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/comeBack/comeBack.exe")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/applicationsBin/myAppSC" TYPE EXECUTABLE FILES "C:/Users/anton/VisibleSim/build-mingw64/myAppSC.exe")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/myAppSC/myAppSC.exe" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/myAppSC/myAppSC.exe")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "C:/msys64/mingw64/bin/strip.exe" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/myAppSC/myAppSC.exe")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/applicationsBin/demo01_SC" TYPE EXECUTABLE FILES "C:/Users/anton/VisibleSim/build-mingw64/demo01_SC.exe")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/demo01_SC/demo01_SC.exe" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/demo01_SC/demo01_SC.exe")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "C:/msys64/mingw64/bin/strip.exe" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/demo01_SC/demo01_SC.exe")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/applicationsBin/catoms3DRotate" TYPE EXECUTABLE FILES "C:/Users/anton/VisibleSim/build-mingw64/catoms3DRotate.exe")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/catoms3DRotate/catoms3DRotate.exe" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/catoms3DRotate/catoms3DRotate.exe")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "C:/msys64/mingw64/bin/strip.exe" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/catoms3DRotate/catoms3DRotate.exe")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/applicationsBin/flowBridgeCatoms3D" TYPE EXECUTABLE FILES "C:/Users/anton/VisibleSim/build-mingw64/flowBridgeCatoms3D.exe")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/flowBridgeCatoms3D/flowBridgeCatoms3D.exe" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/flowBridgeCatoms3D/flowBridgeCatoms3D.exe")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "C:/msys64/mingw64/bin/strip.exe" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/flowBridgeCatoms3D/flowBridgeCatoms3D.exe")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/applicationsBin/testCase" TYPE EXECUTABLE FILES "C:/Users/anton/VisibleSim/build-mingw64/testCase.exe")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/testCase/testCase.exe" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/testCase/testCase.exe")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "C:/msys64/mingw64/bin/strip.exe" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/testCase/testCase.exe")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/applicationsBin/catomsCombination" TYPE EXECUTABLE FILES "C:/Users/anton/VisibleSim/build-mingw64/catomsCombination.exe")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/catomsCombination/catomsCombination.exe" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/catomsCombination/catomsCombination.exe")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "C:/msys64/mingw64/bin/strip.exe" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/catomsCombination/catomsCombination.exe")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/applicationsBin/e1_flooding" TYPE EXECUTABLE FILES "C:/Users/anton/VisibleSim/build-mingw64/e1_flooding.exe")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/e1_flooding/e1_flooding.exe" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/e1_flooding/e1_flooding.exe")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "C:/msys64/mingw64/bin/strip.exe" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/e1_flooding/e1_flooding.exe")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/applicationsBin/e2_goback" TYPE EXECUTABLE FILES "C:/Users/anton/VisibleSim/build-mingw64/e2_goback.exe")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/e2_goback/e2_goback.exe" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/e2_goback/e2_goback.exe")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "C:/msys64/mingw64/bin/strip.exe" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/e2_goback/e2_goback.exe")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/applicationsBin/e3_lightpath" TYPE EXECUTABLE FILES "C:/Users/anton/VisibleSim/build-mingw64/e3_lightpath.exe")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/e3_lightpath/e3_lightpath.exe" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/e3_lightpath/e3_lightpath.exe")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "C:/msys64/mingw64/bin/strip.exe" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/e3_lightpath/e3_lightpath.exe")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/applicationsBin/borders" TYPE EXECUTABLE FILES "C:/Users/anton/VisibleSim/build-mingw64/borders.exe")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/borders/borders.exe" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/borders/borders.exe")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "C:/msys64/mingw64/bin/strip.exe" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/borders/borders.exe")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/applicationsBin/myAppMotionSC" TYPE EXECUTABLE FILES "C:/Users/anton/VisibleSim/build-mingw64/myAppMotionSC.exe")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/myAppMotionSC/myAppMotionSC.exe" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/myAppMotionSC/myAppMotionSC.exe")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "C:/msys64/mingw64/bin/strip.exe" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/myAppMotionSC/myAppMotionSC.exe")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/applicationsBin/myCSGappBB" TYPE EXECUTABLE FILES "C:/Users/anton/VisibleSim/build-mingw64/myCSGappBB.exe")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/myCSGappBB/myCSGappBB.exe" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/myCSGappBB/myCSGappBB.exe")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "C:/msys64/mingw64/bin/strip.exe" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/myCSGappBB/myCSGappBB.exe")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/applicationsBin/BBbalance" TYPE EXECUTABLE FILES "C:/Users/anton/VisibleSim/build-mingw64/BBbalance.exe")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/BBbalance/BBbalance.exe" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/BBbalance/BBbalance.exe")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "C:/msys64/mingw64/bin/strip.exe" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/applicationsBin/BBbalance/BBbalance.exe")
    endif()
  endif()
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "C:/Users/anton/VisibleSim/build-mingw64/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
if(CMAKE_INSTALL_COMPONENT)
  if(CMAKE_INSTALL_COMPONENT MATCHES "^[a-zA-Z0-9_.+-]+$")
    set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
  else()
    string(MD5 CMAKE_INST_COMP_HASH "${CMAKE_INSTALL_COMPONENT}")
    set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INST_COMP_HASH}.txt")
    unset(CMAKE_INST_COMP_HASH)
  endif()
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "C:/Users/anton/VisibleSim/build-mingw64/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
