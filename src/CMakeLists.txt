ADD_LIBRARY( dynamic_adjustment SHARED
	./selection/pixel_selection.cpp
	./initializer/initializer.cpp
	./camera/camera.cpp
	./frame/frame.cpp
	./frame/frame_pair.cpp
	./loader/frame_loader.cpp
	./optimizer/surround_optimizer.cpp
	./surround/surround_view_system.cpp
	./utils/utils.cpp
)




TARGET_LINK_LIBRARIES( dynamic_adjustment
        ${THIRD_PARTY_LIBS}
)
