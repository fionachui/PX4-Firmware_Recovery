px4_add_module(
	MODULE modules__impact_detection
	MAIN impact_detection
	STACK_MAIN 2000
	COMPILE_FLAGS -Os
	SRCS
		impact_detection.cpp
	DEPENDS
		platforms__common
	)
