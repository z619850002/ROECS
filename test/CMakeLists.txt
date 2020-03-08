ADD_EXECUTABLE(initializer_test initializer_test.cpp)
TARGET_LINK_LIBRARIES( initializer_test dynamic_adjustment)


ADD_EXECUTABLE(direct_unary_edge_test direct_unary_edge_test.cpp)
TARGET_LINK_LIBRARIES( direct_unary_edge_test dynamic_adjustment)


ADD_EXECUTABLE(frame_loader_test frame_loader_test.cpp)
TARGET_LINK_LIBRARIES( frame_loader_test dynamic_adjustment)


ADD_EXECUTABLE(surround_view_test surround_view_test.cpp)
TARGET_LINK_LIBRARIES( surround_view_test dynamic_adjustment)


ADD_EXECUTABLE(surround_multi_test surround_multi_test.cpp)
TARGET_LINK_LIBRARIES( surround_multi_test dynamic_adjustment)


ADD_EXECUTABLE(variance_test variance_test.cpp)
TARGET_LINK_LIBRARIES( variance_test dynamic_adjustment)



ADD_EXECUTABLE(lk_test lk_test.cpp)
TARGET_LINK_LIBRARIES( lk_test dynamic_adjustment)
