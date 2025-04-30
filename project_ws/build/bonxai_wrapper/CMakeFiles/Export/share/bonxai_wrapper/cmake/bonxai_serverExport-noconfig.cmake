#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "bonxai_wrapper::bonxai_server" for configuration ""
set_property(TARGET bonxai_wrapper::bonxai_server APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(bonxai_wrapper::bonxai_server PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libbonxai_server.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS bonxai_wrapper::bonxai_server )
list(APPEND _IMPORT_CHECK_FILES_FOR_bonxai_wrapper::bonxai_server "${_IMPORT_PREFIX}/lib/libbonxai_server.a" )

# Import target "bonxai_wrapper::bonxai_map" for configuration ""
set_property(TARGET bonxai_wrapper::bonxai_map APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(bonxai_wrapper::bonxai_map PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libbonxai_map.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS bonxai_wrapper::bonxai_map )
list(APPEND _IMPORT_CHECK_FILES_FOR_bonxai_wrapper::bonxai_map "${_IMPORT_PREFIX}/lib/libbonxai_map.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
