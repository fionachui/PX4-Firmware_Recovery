
#include <modules/impact_characterization/impact_characterization.h> 
void init_flp_params(Fuzzy*& fuzzy);

void init_flp_params(Fuzzy* &fuzzy){ // to call: init_flp_params(fuzzy);
	// Fuzzy Input 1
	FuzzyInput* accMagHoriz = new FuzzyInput(1);
	FuzzySet* accVeryLow = new FuzzySet(0, 0, 0, 2);
	FuzzySet* accLow = new FuzzySet(0, 2, 2, 4);
	FuzzySet* accMedium = new FuzzySet(2, 4, 4, 6);
	FuzzySet* accHigh = new FuzzySet(4, 6, 10, 10);
	accMagHoriz->addFuzzySet(accVeryLow);
	accMagHoriz->addFuzzySet(accLow);
	accMagHoriz->addFuzzySet(accMedium);
	accMagHoriz->addFuzzySet(accHigh);
	fuzzy->addFuzzyInput(accMagHoriz);
	
	//Fuzzy Input 2
	FuzzyInput* inclination = new FuzzyInput(2);
	FuzzySet* inclinedAwayBig = new FuzzySet(-90, -60, -15, -8);
	FuzzySet* inclinedLevel = new FuzzySet(-2, 0, 0, 2);
	FuzzySet* inclinedTowardBig = new FuzzySet(8, 15, 60, 90);
	FuzzySet* inclinedAwaySmall = new FuzzySet(-9.5, -5.5, -5.5, -1.5);
	FuzzySet* inclinedTowardSmall = new FuzzySet(1.5, 5.5, 5.5, 9.5);
	inclination->addFuzzySet(inclinedAwayBig);
	inclination->addFuzzySet(inclinedLevel);
	inclination->addFuzzySet(inclinedTowardBig);
	inclination->addFuzzySet(inclinedAwaySmall);
	inclination->addFuzzySet(inclinedTowardSmall);
	fuzzy->addFuzzyInput(inclination);

	//Fuzzy Input 3
	FuzzyInput* gamma = new FuzzyInput(3);
	FuzzySet* flipAway = new FuzzySet(0, 0, 50, 90);
	FuzzySet* flipSideway = new FuzzySet(70, 90, 90, 110);
	FuzzySet* flipToward = new FuzzySet(90, 130, 180, 180);
	gamma->addFuzzySet(flipAway);
	gamma->addFuzzySet(flipSideway);
	gamma->addFuzzySet(flipToward);
	fuzzy->addFuzzyInput(gamma);

	//Fuzzy Input 4
	FuzzyInput* gyroHorizMag = new FuzzyInput(4);
	FuzzySet* gyroLow = new FuzzySet(0, 0, 0.5, 1.5);
	FuzzySet* gyroMedium = new FuzzySet(1, 1.5, 3, 3.5);
	FuzzySet* gyroHigh = new FuzzySet(3, 4.5, 15, 15);
	gyroHorizMag->addFuzzySet(gyroLow);
	gyroHorizMag->addFuzzySet(gyroMedium);
	gyroHorizMag->addFuzzySet(gyroHigh);
	fuzzy->addFuzzyInput(gyroHorizMag);	

	//Fuzzy Output: Response Intensity
	FuzzyOutput* responseIntensity = new FuzzyOutput(1);
	FuzzySet* towardBig = new FuzzySet(0.6, 0.9, 1, 1);
	FuzzySet* towardSmall = new FuzzySet(0.1, 0.4, 0.6, 0.9);
	FuzzySet* level = new FuzzySet(-0.4, -0.1, 0.1, 0.4);
	FuzzySet* awaySmall = new FuzzySet(-0.9, -0.6, -0.4, -0.1);
	FuzzySet* awayBig = new FuzzySet(-1, -1, -0.9, -0.6);
	responseIntensity->addFuzzySet(towardBig);
	responseIntensity->addFuzzySet(towardSmall);
	responseIntensity->addFuzzySet(level);
	responseIntensity->addFuzzySet(awaySmall);
	responseIntensity->addFuzzySet(awayBig);
	fuzzy->addFuzzyOutput(responseIntensity);

	// Build Fuzzy Rules:
	// Consequences:
	FuzzyRuleConsequent* thenLevel = new FuzzyRuleConsequent();
	thenLevel->addOutput(level);
	
	FuzzyRuleConsequent* thenTowardSmall = new FuzzyRuleConsequent();
	thenTowardSmall->addOutput(towardSmall);
		
	FuzzyRuleConsequent* thenAwaySmall = new FuzzyRuleConsequent();
	thenAwaySmall->addOutput(awaySmall);
		
	FuzzyRuleConsequent* thenTowardBig = new FuzzyRuleConsequent();
	thenTowardBig->addOutput(towardBig);
			
	FuzzyRuleConsequent* thenAwayBig = new FuzzyRuleConsequent();
	thenAwayBig->addOutput(awayBig);

	// // Build Rule Set 1
	// General Rules
	FuzzyRuleAntecedent* ifInclinedToward = new FuzzyRuleAntecedent();
	ifInclinedToward->joinWithOR(inclinedTowardSmall, inclinedTowardBig);

	FuzzyRuleAntecedent* ifInclinedAway = new FuzzyRuleAntecedent();
	ifInclinedAway->joinWithOR(inclinedAwaySmall, inclinedAwayBig);
	
	// Rule Set 1, Rule 1 (Level Output)
	FuzzyRuleAntecedent* ifAccVeryLowOrInclinedLevel = new FuzzyRuleAntecedent();
	ifAccVeryLowOrInclinedLevel->joinWithOR(accVeryLow, inclinedLevel);	
	
	FuzzyRule* fuzzyRuleSet1Rule1 = new FuzzyRule(1,ifAccVeryLowOrInclinedLevel,thenLevel);
	fuzzy->addFuzzyRule(fuzzyRuleSet1Rule1);

	// Rule Set 1, Rule 2 (Toward Small Output)
	FuzzyRuleAntecedent* ifAccLowAndInclinedToward = new FuzzyRuleAntecedent();
	ifAccLowAndInclinedToward->joinWithAND(accLow, ifInclinedToward);	
	
	FuzzyRuleAntecedent* ifAccMedAndInclinedTS = new FuzzyRuleAntecedent();
	ifAccMedAndInclinedTS->joinWithAND(accMedium, inclinedTowardSmall);	
	
	FuzzyRuleAntecedent* ifTowardSmallConditions = new FuzzyRuleAntecedent();
	ifTowardSmallConditions->joinWithOR(ifAccLowAndInclinedToward,ifAccMedAndInclinedTS); 
	
	FuzzyRule* fuzzyRuleSet1Rule2 = new FuzzyRule(2, ifTowardSmallConditions,thenTowardSmall);
	fuzzy->addFuzzyRule(fuzzyRuleSet1Rule2);
	
	//Rule Set 1, Rule 3 (Away Small Output)
	FuzzyRuleAntecedent* ifAccLowAndInclinedAway = new FuzzyRuleAntecedent();
	ifAccLowAndInclinedAway->joinWithAND(accLow, ifInclinedAway);	
	
	FuzzyRuleAntecedent* ifAccMedAndInclinedAS = new FuzzyRuleAntecedent();
	ifAccMedAndInclinedAS->joinWithAND(accMedium, inclinedAwaySmall);	
	
	FuzzyRuleAntecedent* ifAwaySmallConditions = new FuzzyRuleAntecedent();
	ifAwaySmallConditions->joinWithOR(ifAccLowAndInclinedAway,ifAccMedAndInclinedAS); 

	FuzzyRule* fuzzyRuleSet1Rule3 = new FuzzyRule(3, ifAwaySmallConditions,thenAwaySmall);
	fuzzy->addFuzzyRule(fuzzyRuleSet1Rule3);
	
	// Rule Set 1, Rule 4 (Toward Big Output)
	FuzzyRuleAntecedent* ifAccHighAndInclinedToward = new FuzzyRuleAntecedent();
	ifAccHighAndInclinedToward->joinWithAND(accHigh, ifInclinedToward);	
	
	FuzzyRuleAntecedent* ifAccMedAndInclinedTB = new FuzzyRuleAntecedent();
	ifAccMedAndInclinedTB->joinWithAND(accMedium, inclinedTowardBig);	
	
	FuzzyRuleAntecedent* ifTowardBigConditions = new FuzzyRuleAntecedent();
	ifTowardBigConditions->joinWithOR(ifAccHighAndInclinedToward,ifAccMedAndInclinedTB); 

	FuzzyRule* fuzzyRuleSet1Rule4 = new FuzzyRule(4, ifTowardBigConditions,thenTowardBig);
	fuzzy->addFuzzyRule(fuzzyRuleSet1Rule4);
	
	// Rule Set 1, Rule 5 (Away Big Output)
	FuzzyRuleAntecedent* ifAccHighAndInclinedAway = new FuzzyRuleAntecedent();
	ifAccHighAndInclinedAway->joinWithAND(accHigh, ifInclinedAway);	
	
	FuzzyRuleAntecedent* ifAccMedAndInclinedAB = new FuzzyRuleAntecedent();
	ifAccMedAndInclinedAB->joinWithAND(accMedium, inclinedAwayBig);	
	
	FuzzyRuleAntecedent* ifAwayBigConditions = new FuzzyRuleAntecedent();
	ifAwayBigConditions->joinWithOR(ifAccHighAndInclinedAway,ifAccMedAndInclinedAB); 

	FuzzyRule* fuzzyRuleSet1Rule5 = new FuzzyRule(5, ifAwayBigConditions,thenAwayBig);
	fuzzy->addFuzzyRule(fuzzyRuleSet1Rule5);
	
	// Build Fuzzy Rule Set 2
	// Rule Set 2, Rule 1 (Level Outpu)
	FuzzyRuleAntecedent* ifGyroLow = new FuzzyRuleAntecedent();
	ifGyroLow->joinSingle(gyroLow);
	
	FuzzyRule* fuzzyRuleSet2Rule1 = new FuzzyRule(6, ifGyroLow, thenLevel);
	fuzzy->addFuzzyRule(fuzzyRuleSet2Rule1);
	
	// Rule Set 2, Rule 2 (Toward Small Output)
	FuzzyRuleAntecedent* ifGyroMedAndFlipToward = new FuzzyRuleAntecedent();
	ifGyroMedAndFlipToward->joinWithAND(gyroMedium,flipToward);
	
	FuzzyRule* fuzzyRuleSet2Rule2 = new FuzzyRule(7, ifGyroMedAndFlipToward, thenTowardSmall);
	fuzzy->addFuzzyRule(fuzzyRuleSet2Rule2);
	
	// Rule Set 2, Rule 3 (Away Small Output)
	FuzzyRuleAntecedent* ifGyroMedAndFlipAway = new FuzzyRuleAntecedent();
	ifGyroMedAndFlipAway->joinWithAND(gyroMedium,flipAway);
	
	FuzzyRule* fuzzyRuleSet2Rule3 = new FuzzyRule(8, ifGyroMedAndFlipAway, thenAwaySmall);
	fuzzy->addFuzzyRule(fuzzyRuleSet2Rule3);	
	
	// Rule Set 2, Rule 4 (Toward Big Output)
	FuzzyRuleAntecedent* ifGyroHighAndFlipToward = new FuzzyRuleAntecedent();
	ifGyroHighAndFlipToward->joinWithAND(gyroHigh,flipToward);
	
	FuzzyRule* fuzzyRuleSet2Rule4 = new FuzzyRule(9, ifGyroHighAndFlipToward, thenTowardBig);
	fuzzy->addFuzzyRule(fuzzyRuleSet2Rule4);
	
	// Rule Set 2, Rule 5 (Away Big Output)
	FuzzyRuleAntecedent* ifGyroHighAndFlipAway = new FuzzyRuleAntecedent();
	ifGyroHighAndFlipAway->joinWithAND(gyroHigh,flipAway);
	
	FuzzyRule* fuzzyRuleSet2Rule5 = new FuzzyRule(10, ifGyroHighAndFlipAway, thenAwayBig);
	fuzzy->addFuzzyRule(fuzzyRuleSet2Rule5);	
}

