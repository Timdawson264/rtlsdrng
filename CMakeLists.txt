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
    onion
    sox
    microhttpd
    json-c
)
