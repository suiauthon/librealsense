macro(add_camerasuite)
    target_include_directories(${LRS_TARGET} PRIVATE ${CAMERASUITE_INCLUDE_DIRS})
    target_link_libraries(${LRS_TARGET} PRIVATE ${CAMERASUITE_LIBRARY})
endmacro()