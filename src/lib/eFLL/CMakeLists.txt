px4_add_module(
	MODULE lib__eFLL
	COMPILE_FLAGS -Os -w
	SRCS
		FuzzyComposition.cpp
		Fuzzy.cpp
		FuzzyInput.cpp
		FuzzyIO.cpp
		FuzzyOutput.cpp
		FuzzyRuleAntecedent.cpp
		FuzzyRuleConsequent.cpp
		FuzzyRule.cpp
		FuzzySet.cpp

	DEPENDS
		platforms__common
	)
