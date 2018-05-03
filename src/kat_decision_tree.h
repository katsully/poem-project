
/*
This inline function was automatically generated using DecisionTreeToCpp Converter

It takes feature vector as single argument:
feature_vector[0] - feature0
feature_vector[1] - feature1
feature_vector[2] - feature2
feature_vector[3] - feature3
feature_vector[4] - feature4
feature_vector[5] - feature5
feature_vector[6] - feature6
feature_vector[7] - feature7
feature_vector[8] - feature8
feature_vector[9] - feature9
feature_vector[10] - feature10
feature_vector[11] - feature11
feature_vector[12] - feature12
feature_vector[13] - feature13
feature_vector[14] - feature14
feature_vector[15] - feature15
feature_vector[16] - feature16
feature_vector[17] - feature17
feature_vector[18] - feature18
feature_vector[19] - feature19
feature_vector[20] - feature20
feature_vector[21] - feature21


It returns index of predicted class:
0 - Pose 1
1 - Pose 2
2 - Pose 3
3 - Pose 4
4 - Pose 5


Simply include this file to your project and use it
*/

#include <vector>

inline int kat_decision_tree(const std::vector<double> & feature_vector)
{
	if (feature_vector.at(14) <= 0.54) {
		if (feature_vector.at(11) <= 0.33) {
			if (feature_vector.at(20) <= -0.2) {
				return 3;
			}
			else {
				return 0;
			}
		}
		else {
			if (feature_vector.at(11) <= 0.54) {
				return 4;
			}
			else {
				if (feature_vector.at(14) <= 0.41) {
					return 1;
				}
				else {
					return 2;
				}
			}
		}
	}
	else {
		return 2;
	}
}