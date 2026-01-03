target_include_directories(${PROJECT_NAME} PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/Core/Inc"
    "${CMAKE_CURRENT_LIST_DIR}/DSP/Inc/dsp"
    "${CMAKE_CURRENT_LIST_DIR}/DSP/Inc"
    "${CMAKE_CURRENT_LIST_DIR}/DSP/PrivateInclude"
)

target_sources(${PROJECT_NAME} PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/DSP/Src/BasicMathFunctions/BasicMathFunctions.c"
    "${CMAKE_CURRENT_LIST_DIR}/DSP/Src/BayesFunctions/BayesFunctions.c"
    "${CMAKE_CURRENT_LIST_DIR}/DSP/Src/CommonTables/CommonTables.c"
    "${CMAKE_CURRENT_LIST_DIR}/DSP/Src/ComplexMathFunctions/ComplexMathFunctions.c"
    "${CMAKE_CURRENT_LIST_DIR}/DSP/Src/ControllerFunctions/ControllerFunctions.c"
    "${CMAKE_CURRENT_LIST_DIR}/DSP/Src/DistanceFunctions/DistanceFunctions.c"
    "${CMAKE_CURRENT_LIST_DIR}/DSP/Src/FastMathFunctions/FastMathFunctions.c"
    "${CMAKE_CURRENT_LIST_DIR}/DSP/Src/FilteringFunctions/FilteringFunctions.c"
    "${CMAKE_CURRENT_LIST_DIR}/DSP/Src/InterpolationFunctions/InterpolationFunctions.c"
    "${CMAKE_CURRENT_LIST_DIR}/DSP/Src/MatrixFunctions/MatrixFunctions.c"
    "${CMAKE_CURRENT_LIST_DIR}/DSP/Src/QuaternionMathFunctions/QuaternionMathFunctions.c"
    "${CMAKE_CURRENT_LIST_DIR}/DSP/Src/StatisticsFunctions/StatisticsFunctions.c"
    "${CMAKE_CURRENT_LIST_DIR}/DSP/Src/SupportFunctions/SupportFunctions.c"
    "${CMAKE_CURRENT_LIST_DIR}/DSP/Src/SVMFunctions/SVMFunctions.c"
    "${CMAKE_CURRENT_LIST_DIR}/DSP/Src/TransformFunctions/TransformFunctions.c"
    "${CMAKE_CURRENT_LIST_DIR}/DSP/Src/WindowFunctions/WindowFunctions.c"
)
