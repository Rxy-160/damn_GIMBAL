target_include_directories(${PROJECT_NAME} PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/Algorithm/Inc"
    "${CMAKE_CURRENT_LIST_DIR}/App/Inc"
    "${CMAKE_CURRENT_LIST_DIR}/Bsp/Inc"
    "${CMAKE_CURRENT_LIST_DIR}/Start/Inc"
    "${CMAKE_CURRENT_LIST_DIR}/UI"
)

aux_source_directory("${CMAKE_CURRENT_LIST_DIR}/Algorithm" Algorithm_Src)
target_sources(${PROJECT_NAME} PRIVATE ${Algorithm_Src})

aux_source_directory("${CMAKE_CURRENT_LIST_DIR}/App" App_Src)
target_sources(${PROJECT_NAME} PRIVATE ${App_Src})

aux_source_directory("${CMAKE_CURRENT_LIST_DIR}/Bsp" Bsp_Src)
target_sources(${PROJECT_NAME} PRIVATE ${Bsp_Src})

aux_source_directory("${CMAKE_CURRENT_LIST_DIR}/Start" Start_Src)
target_sources(${PROJECT_NAME} PRIVATE ${Start_Src})

aux_source_directory("${CMAKE_CURRENT_LIST_DIR}/UI" UI_Src)
target_sources(${PROJECT_NAME} PRIVATE ${UI_Src})

