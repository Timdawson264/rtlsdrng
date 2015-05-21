cmake_minimum_required (VERSION 2.6)
project (rtlsdrng)

#PRINT Commands
set(CMAKE_VERBOSE_MAKEFILE on)

########################################################################
# Setup libraries used in executables
########################################################################
add_library(convenience_static STATIC
    convenience/convenience.c
   convenience/CirDLinkedList.c
)

#add library for common functions
add_library(fm_httpd fm_httpd_common.c)

#Setting up ftdi library
execute_process(COMMAND libftdi-config --libs COMMAND tr -d '\n'  OUTPUT_VARIABLE FTDILIBS)
SET( CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${FTDILIBS}" )
execute_process(COMMAND libftdi1-config --libs COMMAND tr -d '\n'  OUTPUT_VARIABLE FTDILIBS)
SET( CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${FTDILIBS}" )

########################################################################
# Build utility
########################################################################
add_executable(rtl_httpd rtl_httpd.c)
add_executable(rtl_fm rtl_fm.c)
add_executable(ftdi_dss dss.c)
add_executable(rtlsdr_test rtlsdr_test.c)



target_link_libraries(ftdi_dss
    m
)
  

target_link_libraries(rtl_fm
    fm_httpd
    convenience_static
    pthread
    rtlsdr
    m
)

target_link_libraries(rtl_httpd
    fm_httpd
    convenience_static
    pthread
    rtlsdr
    m
    sox
    microhttpd
    json-c
)

IF (UNIX)
  ADD_CUSTOM_TARGET (distclean @echo cleaning for source distribution)
  SET(DISTCLEANED
   cmake.depends
   cmake.check_depends
   CMakeCache.txt
   cmake.check_cache
   *.cmake
   Makefile
   core core.*
   gmon.out
   *~
   CMakeFiles
  )
  
  ADD_CUSTOM_COMMAND(
    DEPENDS clean
    COMMENT "distribution clean"
    COMMAND rm
    ARGS    -Rf CMakeTmp ${DISTCLEANED}
    TARGET  distclean
  )
  
ENDIF(UNIX)
