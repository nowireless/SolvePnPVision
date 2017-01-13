//
// Created by ryan on 1/5/17.
//

#include "Target.h"

using namespace cv;
using namespace std;
using namespace vision;

//Initialize constants here
//Note: That aspect ratios are long side / short side
const Target Target::kNOT_A_TARGET      = Target(TargetID::kNotATarget, 1);
const Target Target::kHORIZONTAL_TARGET = Target(TargetID::kHorizontal, 12.0/4.0);
const Target Target::kVERTICAL_TARGET   = Target(TargetID::kVertical, 12.0/4.0);

const vector<Target> Target::TARGETS = Target::InitTargets();

vector<Target> Target::InitTargets() {
    vector<Target> ret;
    ret.push_back(kNOT_A_TARGET);
    ret.push_back(kHORIZONTAL_TARGET);
    ret.push_back(kVERTICAL_TARGET);
    return ret;
}