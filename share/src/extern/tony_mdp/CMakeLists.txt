get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

find_package(BISON)
BISON_TARGET(mdp_parser 
  parser.ypp
  ${CMAKE_CURRENT_BINARY_DIR}/parser.tab.cpp)

find_package(FLEX)
FLEX_TARGET(mdp_scanner 
  scanner.l 
  ${CMAKE_CURRENT_BINARY_DIR}/lex.yy.c)

ADD_FLEX_BISON_DEPENDENCY(mdp_scanner mdp_parser)

include_directories(${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR})

# for whatever reason, g++ compiles it, while gcc doesn't
set_source_files_properties(${CMAKE_CURRENT_BINARY_DIR}/lex.yy.c PROPERTIES LANGUAGE CXX)

set(SOURCES ${FLEX_mdp_scanner_OUTPUTS} ${BISON_mdp_parser_OUTPUTS}
  imm-reward.cpp
  parse_err.cpp
  parse_hash.cpp
  sparse-matrix.cpp
  mdp.cpp)

add_library(extern_${NAME} STATIC ${SOURCES})
set_property(TARGET extern_${NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)

