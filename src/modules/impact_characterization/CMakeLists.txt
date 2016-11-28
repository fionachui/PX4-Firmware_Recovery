px4_add_module(
	MODULE modules__impact_characterization
	MAIN impact_characterization
	STACK_MAIN 2000
	COMPILE_FLAGS -Os
	SRCS
		initialize_fuzzylogicprocess.cpp
		impact_characterization.cpp
	DEPENDS
		platforms__common
	)
