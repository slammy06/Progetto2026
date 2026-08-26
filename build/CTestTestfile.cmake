# CMake generated Testfile for 
# Source directory: /home/diego/esame/progetto
# Build directory: /home/diego/esame/progetto/build
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
if(CTEST_CONFIGURATION_TYPE MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
  add_test([=[run.t]=] "/home/diego/esame/progetto/build/Debug/run.t")
  set_tests_properties([=[run.t]=] PROPERTIES  _BACKTRACE_TRIPLES "/home/diego/esame/progetto/CMakeLists.txt;53;add_test;/home/diego/esame/progetto/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
  add_test([=[run.t]=] "/home/diego/esame/progetto/build/Release/run.t")
  set_tests_properties([=[run.t]=] PROPERTIES  _BACKTRACE_TRIPLES "/home/diego/esame/progetto/CMakeLists.txt;53;add_test;/home/diego/esame/progetto/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
  add_test([=[run.t]=] "/home/diego/esame/progetto/build/RelWithDebInfo/run.t")
  set_tests_properties([=[run.t]=] PROPERTIES  _BACKTRACE_TRIPLES "/home/diego/esame/progetto/CMakeLists.txt;53;add_test;/home/diego/esame/progetto/CMakeLists.txt;0;")
else()
  add_test([=[run.t]=] NOT_AVAILABLE)
endif()
