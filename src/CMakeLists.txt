ADD_LIBRARY( dynamic_adjustment SHARED
	./initializer/initializer.cpp
	./camera/camera.cpp
	./frame/frame.cpp
	./frame/frame_pair.cpp
	./loader/frame_loader.cpp
	./surround/surround_view_system.cpp
)




TARGET_LINK_LIBRARIES( dynamic_adjustment
        ${THIRD_PARTY_LIBS}
)
