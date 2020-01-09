ADD_LIBRARY( dynamic_adjustment SHARED
	initializer.cpp
	camera.cpp
)




TARGET_LINK_LIBRARIES( dynamic_adjustment
        ${THIRD_PARTY_LIBS}
)
