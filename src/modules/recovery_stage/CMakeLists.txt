px4_add_module(
	MODULE modules__recovery_stage
	MAIN recovery_stage
	STACK_MAIN 2000
	COMPILE_FLAGS -Os
	SRCS
		recovery_stage.cpp
	DEPENDS
		platforms__common
	)
