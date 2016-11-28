#impact_characterization
float32[4] fuzzyInput		# four fuzzy inputs: accelerometer spike, inclination, pointing direction, and spin magnitude
float32    fuzzyOutput		# fuzzy logic output ranging between -1 and 1
float32[3] wallNormal		# the estimated vector pointing horizontally away from the impact
float32[3] accelReference       # reference acceleration for recovery control post impact
bool accelRefIsComputed         # flag that the impact characterization output has been computed
