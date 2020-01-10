ADD_LIBRARY( dynamic_adjustment SHARED
	./initializer/initializer.cpp
	./camera/camera.cpp
	./frame/frame.cpp
	./frame/frame_pair.cpp
	./loader/frame_loader.cpp
)




TARGET_LINK_LIBRARIES( dynamic_adjustment
        ${THIRD_PARTY_LIBS}
)
