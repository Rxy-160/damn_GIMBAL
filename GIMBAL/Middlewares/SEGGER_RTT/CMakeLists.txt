target_include_directories(${PROJECT_NAME} PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/Config"
    "${CMAKE_CURRENT_LIST_DIR}/RTT"
)

aux_source_directory("${CMAKE_CURRENT_LIST_DIR}/RTT" RTT_Src)
target_sources(${PROJECT_NAME} PRIVATE ${RTT_Src})

