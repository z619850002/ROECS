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


ADD_EXECUTABLE(homography_test homography_test.cpp)
TARGET_LINK_LIBRARIES( homography_test dynamic_adjustment)



ADD_EXECUTABLE(all_surround_view_test all_surround_view_test.cpp)
TARGET_LINK_LIBRARIES( all_surround_view_test dynamic_adjustment)


ADD_EXECUTABLE(all_OECS all_OECS.cpp)
TARGET_LINK_LIBRARIES( all_OECS dynamic_adjustment)



ADD_EXECUTABLE(inverse_depth_test inverse_depth_test.cpp)
TARGET_LINK_LIBRARIES( inverse_depth_test dynamic_adjustment)

ADD_EXECUTABLE(matcher_test matcher_test.cpp)
TARGET_LINK_LIBRARIES( matcher_test dynamic_adjustment)



ADD_EXECUTABLE(offline_baseline_test offline_baseline_test.cpp)
TARGET_LINK_LIBRARIES( offline_baseline_test dynamic_adjustment)
