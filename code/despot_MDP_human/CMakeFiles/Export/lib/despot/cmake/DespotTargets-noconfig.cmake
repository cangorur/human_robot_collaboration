#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "despot" for configuration ""
set_property(TARGET despot APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(despot PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_NOCONFIG "/usr/lib/x86_64-linux-gnu/libboost_system.so;/usr/lib/x86_64-linux-gnu/libboost_coroutine.so;/usr/lib/x86_64-linux-gnu/libboost_context.so;/usr/lib/x86_64-linux-gnu/libboost_thread.so;/usr/lib/x86_64-linux-gnu/libboost_chrono.so;/usr/lib/x86_64-linux-gnu/libboost_date_time.so;/usr/lib/x86_64-linux-gnu/libboost_atomic.so;/usr/lib/x86_64-linux-gnu/libpthread.so;/usr/lib/x86_64-linux-gnu/libcrypto.so;-lpthread"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libdespot.so"
  IMPORTED_SONAME_NOCONFIG "libdespot.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS despot )
list(APPEND _IMPORT_CHECK_FILES_FOR_despot "${_IMPORT_PREFIX}/lib/libdespot.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
