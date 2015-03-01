cmake_minimum_required (VERSION 2.6)
project (rtlsdrng)

########################################################################
# Setup libraries used in executables
########################################################################
add_library(convenience_static STATIC
    convenience/convenience.c
   convenience/CirDLinkedList.c
)

#add library for common functions
add_library(fm_httpd fm_httpd_common.c)


########################################################################
# Build utility
########################################################################
add_executable(rtl_httpd rtl_httpd.c)
add_executable(rtl_fm rtl_fm.c)
add_executable(ftdi_dss dss.c)
add_executable(rtlsdr_test rtlsdr_test.c)

target_link_libraries(ftdi_dss
    ftdi1
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
    onion
    microhttpd
    json-c
)
