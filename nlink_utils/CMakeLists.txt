find_package(serial REQUIRED)
add_library(nlink_utils STATIC)
target_sources(nlink_utils PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/src/init_serial.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/nlink_protocol.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/nutils.cpp
        ${CMAKE_CURRENT_LIST_DIR}/nlink_unpack/nlink_utils.c
        ${CMAKE_CURRENT_LIST_DIR}/nlink_unpack/nlink_linktrack_anchorframe0.c
        ${CMAKE_CURRENT_LIST_DIR}/nlink_unpack/nlink_linktrack_tagframe0.c
        ${CMAKE_CURRENT_LIST_DIR}/nlink_unpack/nlink_linktrack_nodeframe0.c
        ${CMAKE_CURRENT_LIST_DIR}/nlink_unpack/nlink_linktrack_nodeframe1.c
        ${CMAKE_CURRENT_LIST_DIR}/nlink_unpack/nlink_linktrack_nodeframe2.c
        ${CMAKE_CURRENT_LIST_DIR}/nlink_unpack/nlink_linktrack_nodeframe3.c
        ${CMAKE_CURRENT_LIST_DIR}/nlink_unpack/nlink_linktrack_nodeframe4.c
        ${CMAKE_CURRENT_LIST_DIR}/nlink_unpack/nlink_linktrack_nodeframe5.c
        ${CMAKE_CURRENT_LIST_DIR}/nlink_unpack/nlink_linktrack_nodeframe6.c
        ${CMAKE_CURRENT_LIST_DIR}/nlink_unpack/nlink_tofsense_frame0.c
        ${CMAKE_CURRENT_LIST_DIR}/nlink_unpack/nlink_tofsensem_frame0.c
        ${CMAKE_CURRENT_LIST_DIR}/nlink_unpack/nlink_linktrack_aoa_nodeframe0.c
        ${CMAKE_CURRENT_LIST_DIR}/nlink_unpack/nlink_iot_frame0.c
        ${CMAKE_CURRENT_LIST_DIR}/protocol_extracter/nprotocol_base.cpp
        ${CMAKE_CURRENT_LIST_DIR}/protocol_extracter/nprotocol_extracter.cpp
)
target_include_directories(nlink_utils
        PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/include/nlink_utils
        ${CMAKE_CURRENT_LIST_DIR}/nlink_unpack
        ${CMAKE_CURRENT_LIST_DIR}/protocol_extracter
        ${CMAKE_CURRENT_LIST_DIR}
)
target_link_libraries(nlink_utils
        PUBLIC
        serial
)