#
# Copyright (c) 2020 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

#
# Helper macro for verifying that at least one of the required arguments has
# been provided by the caller.
#
# As FATAL_ERROR will be raised if not one of the required arguments has been
# passed by the caller.
#
# Usage:
#   check_arguments_required(<function_name> <prefix> <arg1> [<arg2> ...])
#
macro(check_arguments_required function prefix)
  set(required_found FALSE)
  foreach(required ${ARGN})
    if(DEFINED ${prefix}_${required})
      set(required_found TRUE)
    endif()
  endforeach()

  if(NOT required_found)
    message(FATAL_ERROR "${function}(...) missing a required argument: ${ARGN}")
  endif()
endmacro()

#
# Helper macro for verifying that all the required arguments has # been
# provided by the caller.
#
# As FATAL_ERROR will be raised if one of the required arguments is missing.
#
# Usage:
#   check_arguments_required_all(<function_name> <prefix> <arg1> [<arg2> ...])
#
macro(check_arguments_required_all function prefix)
  foreach(required ${ARGN})
    if(NOT DEFINED ${prefix}_${required})
      message(FATAL_ERROR "${function}(...) missing a required argument: ${required}")
    endif()
  endforeach()
endmacro()

#
# Helper macro for verifying that none of the mutual exclusive arguments are
# provided together with the first argument.
#
# As FATAL_ERROR will be raised if first argument is given together with one
# of the following mutual exclusive arguments.
#
# Usage:
#   check_arguments_exclusive(<function_name> <prefix> <arg1> <exlude-arg1> [<exclude-arg2> ...])
#
macro(check_arguments_exclusive function prefix argument)
  foreach(prohibited ${ARGN})
    if(DEFINED ${prefix}_${argument} AND ${prefix}_${prohibited})
      message(FATAL_ERROR "set_shared(${argument} ...) cannot be used with "
        "argument: ${prohibited}"
      )
    endif()
  endforeach()
endmacro()

function(get_board_without_ns_suffix board_in board_out)
  string(REGEX REPLACE "((_|/)?ns)$" "" board_in_without_suffix ${board_in})
  if(NOT "${board_in}" STREQUAL "${board_in_without_suffix}")
    if (NOT CONFIG_ARM_NONSECURE_FIRMWARE)
      message(FATAL_ERROR "${board_in} is not a valid name for a board without "
      "'CONFIG_ARM_NONSECURE_FIRMWARE' set. This because the 'ns'/'_ns' ending "
      "indicates that the board is the non-secure variant in a TrustZone "
      "enabled system.")
    endif()
    set(${board_out} ${board_in_without_suffix} PARENT_SCOPE)
    message("Changed board to secure ${board_in_without_suffix} (NOT NS)")
  else()
    set(${board_out} ${board_in} PARENT_SCOPE)
  endif()
endfunction()

# Add a partition manager configuration file to the build.
function(ncs_add_partition_manager_config config_file)
  get_filename_component(pm_path ${config_file} REALPATH)
  get_filename_component(pm_filename ${config_file} NAME)

  if (NOT EXISTS ${pm_path})
    message(FATAL_ERROR
      "Could not find specified partition manager configuration file "
      "${config_file} at ${pm_path}"
      )
  endif()

  set_property(GLOBAL APPEND PROPERTY
    PM_SUBSYS_PATHS
    ${pm_path}
    )
  set_property(GLOBAL APPEND PROPERTY
    PM_SUBSYS_OUTPUT_PATHS
    ${CMAKE_CURRENT_BINARY_DIR}/${pm_filename}
    )
endfunction()

# Usage:
#   ncs_file(<mode> <arg> ...)
#
# NCS file function extension.
# This function extends the zephyr_file(CONF_FILES <arg>) function to support
# switching BOARD for images.
#
# It also supports lookup of static partition manager files for based on
# the board name, revision, and the current build type.
# The order at which files are considers is from the most specific to the least specific:
# - first, file name with board name, board revision and build type identifiers,
# - second, file name with board name and build type identifiers,
# - third, file with only build type identifier,
# - finally, the file with no identifiers is looked up.
# During each pass, if domain is defined, the file with additional domain identifier has precedence.
#
# This function currently support the following <modes>.
#
# BOARD <board>: Board name to use when searching for board specific Kconfig
#                fragments.
#
# CONF_FILES <path>: Find all configuration files in path and return them in a
#                    list. Configuration files will be:
#                    - DTS:       Overlay files (.overlay)
#                    - Kconfig:   Config fragments (.conf)
#                    The conf file search will return existing configuration
#                    files for BOARD or the current board if BOARD argument is
#                    not given.
#                    CONF_FILES takes the following additional arguments:
#                    BOARD <board>:             Find configuration files for specified board.
#                    BOARD_REVISION <revision>: Find configuration files for specified board
#                                               revision. Requires BOARD to be specified.
#
#                                               If no board is given the current BOARD and
#                                               BOARD_REVISION will be used.
#
#                    DTS <list>:   List to populate with DTS overlay files
#                    KCONF <list>: List to populate with Kconfig fragment files
#                    PM <list>:    List to populate with board / domain specific
#                                  static partition manager files
#                    DOMAIN <domain>: Domain to use. This argument is only effective
#                                     for partition manager configuration files.
#
function(ncs_file)
  set(file_options CONF_FILES)
  if((ARGC EQUAL 0) OR (NOT (ARGV0 IN_LIST file_options)))
    message(FATAL_ERROR "No <mode> given to `ncs_file(<mode> <args>...)` function,\n \
Please provide one of following: CONF_FILES")
  endif()

  set(single_args CONF_FILES PM DOMAIN)
  set(zephyr_conf_single_args BOARD BOARD_REVISION DTS KCONF SUFFIX)

  cmake_parse_arguments(PREPROCESS_ARGS "" "${single_args};${zephyr_conf_single_args}" "" ${ARGN})
  # Remove any argument that is missing value to ensure proper behavior
  if(DEFINED PREPROCESS_ARGS_KEYWORDS_MISSING_VALUES)
    list(REMOVE_ITEM ARGN ${PREPROCESS_ARGS_KEYWORDS_MISSING_VALUES})
  endif()

  cmake_parse_arguments(NCS_FILE "" "${single_args};BOARD" "" ${ARGN})
  cmake_parse_arguments(ZEPHYR_FILE "" "${zephyr_conf_single_args}" "" ${ARGN})

  if(ZEPHYR_FILE_KCONF)
    if(ZEPHYR_FILE_SUFFIX AND EXISTS ${NCS_FILE_CONF_FILES}/prj_${ZEPHYR_FILE_SUFFIX}.conf)
      set(${ZEPHYR_FILE_KCONF} ${NCS_FILE_CONF_FILES}/prj_${ZEPHYR_FILE_SUFFIX}.conf)
    elseif(EXISTS ${NCS_FILE_CONF_FILES}/prj.conf)
      set(${ZEPHYR_FILE_KCONF} ${NCS_FILE_CONF_FILES}/prj.conf)
    endif()
  endif()

  set(additional_append)

  if(DEFINED PREPROCESS_ARGS_BOARD)
    parse_board_components(PREPROCESS_ARGS_BOARD board_name board_revision board_qualifiers)
    set(additional_append BOARD ${board_name} BOARD_REVISION ${board_revision} BOARD_QUALIFIERS ${board_qualifiers})
  endif()

  zephyr_file(CONF_FILES ${NCS_FILE_CONF_FILES}/boards ${additional_append} ${NCS_FILE_UNPARSED_ARGUMENTS})

  if(ZEPHYR_FILE_KCONF)
    set(${ZEPHYR_FILE_KCONF} ${${ZEPHYR_FILE_KCONF}} PARENT_SCOPE)
  endif()

  if(ZEPHYR_FILE_DTS)
    set(${ZEPHYR_FILE_DTS} ${${ZEPHYR_FILE_DTS}} PARENT_SCOPE)
  endif()

  if(NOT DEFINED PREPROCESS_ARGS_BOARD)
    # Defaulting to system wide settings when BOARD is not given as argument
    set(board_combined ${BOARD})

    if(DEFINED BOARD_REVISION)
      set(board_combined ${board_combined}@${BOARD_REVISION})
    endif()

    set(board_combined ${board_combined}${BOARD_QUALIFIERS})
    parse_board_components(board_combined board_name board_revision board_qualifiers)
  endif()

  if(NCS_FILE_PM)
    set(PM_FILE_PREFIX pm_static)

    # Prepare search for pm_static_board@ver_suffix.yml
    zephyr_build_string(filename
                        BOARD ${board_name}
                        BOARD_REVISION ${board_revision}
                        BOARD_QUALIFIERS ${board_qualifiers}
    )
    set(filename_list ${PM_FILE_PREFIX}_${filename})

    # Prepare search for pm_static_board_suffix.yml
    zephyr_build_string(filename
                        BOARD ${board_name}
                        BOARD_QUALIFIERS ${board_qualifiers}
    )
    list(APPEND filename_list ${PM_FILE_PREFIX}_${filename})
    list(APPEND filename_list ${PM_FILE_PREFIX})
    list(REMOVE_DUPLICATES filename_list)

    foreach(filename ${filename_list})
      if(DEFINED NCS_FILE_DOMAIN)
        if(DEFINED FILE_SUFFIX)
          set(filename_check ${NCS_FILE_CONF_FILES}/${filename}_${NCS_FILE_DOMAIN}.yml)
          zephyr_file_suffix(filename_check SUFFIX ${FILE_SUFFIX})

          if(EXISTS ${filename_check})
            set(${NCS_FILE_PM} ${filename_check} PARENT_SCOPE)
            break()
          endif()

        else()
          if(EXISTS ${NCS_FILE_CONF_FILES}/${filename}_${NCS_FILE_DOMAIN}.yml)
            set(${NCS_FILE_PM} ${NCS_FILE_CONF_FILES}/${filename}_${NCS_FILE_DOMAIN}.yml PARENT_SCOPE)
            break()
          endif()
        endif()
      endif()

      if(DEFINED FILE_SUFFIX)
        set(filename_check ${NCS_FILE_CONF_FILES}/${filename}.yml)
        zephyr_file_suffix(filename_check SUFFIX ${FILE_SUFFIX})
        if(EXISTS ${filename_check})
          set(${NCS_FILE_PM} ${filename_check} PARENT_SCOPE)
          break()
        endif()
      else()
        if(EXISTS ${NCS_FILE_CONF_FILES}/${filename}.yml)
          set(${NCS_FILE_PM} ${NCS_FILE_CONF_FILES}/${filename}.yml PARENT_SCOPE)
          break()
        endif()
      endif()
    endforeach()
  endif()
endfunction()

#
# Usage
#   set_shared(IMAGE <img> [APPEND] PROPERTY <property> <value>)
#
# Shares a property from child to parent.
# The property is shared through an intermediate shared_vars.cmake file which
# will be parsed by the parent image at CMake configure time.
#
# Example usage 'set_shared(IMAGE child PROPERTY visible_in_parent "I AM YOUR CHILD")'
#
# Usage
#   set_shared(FILE <file>)
#
# Shares all properties in file to parent.
# This function can be used to re-share properties from a child to its
# grand parent.
#
function(set_shared)
  set(flags       "APPEND")
  set(single_args "FILE;IMAGE")
  set(multi_args  "PROPERTY")
  cmake_parse_arguments(SHARE "${flags}" "${single_args}" "${multi_args}" ${ARGN})

  list(POP_FRONT SHARE_PROPERTY listname)
  if(SHARE_APPEND)
    list(APPEND ${listname} ${SHARE_PROPERTY})
    list(REMOVE_DUPLICATES ${listname})
    set(SHARE_PROPERTY ${${listname}})
  endif()
  set(${listname} "${SHARE_PROPERTY}" CACHE INTERNAL "shared var")
endfunction()

# generate_shared(IMAGE <img> FILE <file>)
function(generate_shared)
  set(single_args "IMAGE;FILE")
  cmake_parse_arguments(SHARE "" "${single_args}" "" ${ARGN})

  check_arguments_required_all("generate_shared" SHARE IMAGE FILE)

  set(prop_target ${IMAGE_NAME}_shared_property_target)
  file(GENERATE OUTPUT ${SHARE_FILE}
      CONTENT
        "$<JOIN:$<TARGET_PROPERTY:${prop_target},image_targets>,\n>
$<TARGET_PROPERTY:${prop_target},shared_vars>"
    )
endfunction()

#
# Usage
#   get_shared(<var> IMAGE <img> PROPERTY <property>)
#
# Get a property value defined by an image or domain <img> if it exists.
# The property value will be returned in the variable referenced by <var>.
#
# Example usage 'get_shared(prop_value IMAGE child PROPERTY property_in_child)'
#
function(get_shared var)
  set(single_args "IMAGE")
  set(multi_args  "PROPERTY")
  cmake_parse_arguments(SHARE "" "${single_args}" "${multi_args}" ${ARGN})

  check_arguments_required_all("get_shared" SHARE IMAGE PROPERTY)

  if(TARGET ${SHARE_IMAGE}_shared_property_target)
    get_property(
      ${var}
      TARGET   ${SHARE_IMAGE}_shared_property_target
      PROPERTY ${SHARE_PROPERTY}
    )
    set(${var} ${${var}} PARENT_SCOPE)
  endif()
endfunction()

#
# Usage
#   import_pm_config(<dotconf_file> <keys>)
#
# Import variables from a partition manager output .config file
# (usually pm.config or pm_<DOMAIN>.config) into the CMake namespace.
#
# <dotconf_file>: Absolute path to the file.
# <keys_out>:     Output variable, which will be populated with a list
#                 of variable names loaded from <dotconf_file>.
#
function(import_pm_config dotconf_file keys_out)
  file(STRINGS ${dotconf_file} DOTCONF_LIST ENCODING "UTF-8")
  foreach(LINE ${DOTCONF_LIST})
    # Parse `key=value` assignments, where every key is prefixed with `PM_`.
    if("${LINE}" MATCHES "(^PM_[^=]+)=(.*$)")
      set(key "${CMAKE_MATCH_1}")

      # If the value is surrounded by quotation marks, strip them out.
      string(REGEX REPLACE "\"(.*)\"" "\\1" value "${CMAKE_MATCH_2}")

      list(APPEND keys "${key}")
      set("${key}" "${value}" PARENT_SCOPE)
    endif()
  endforeach()
  set("${keys_out}" "${keys}" PARENT_SCOPE)
endfunction()
