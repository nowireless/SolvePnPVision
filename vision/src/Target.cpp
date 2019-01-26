//
// Created by ryan on 1/5/17.
//

#include <vision/Target.h>

using namespace cv;
using namespace std;
using namespace vision;

//Initialize constants here
//Note: That aspect ratios are long side / short side
const Target Target::kNOT_A_TARGET      = Target(TargetID::kNotATarget, 1, "Not a target");
const Target Target::kLEFT_TILT_TARGET = Target(TargetID::kLeftTilt, 5.5/2.0, "Left tilt target");
const Target Target::kRIGHT_TILT_TARGET = Target(TargetID::kRightTilt, 5.5/2.0, "Right tilt target");

const vector<Target> Target::TARGETS = Target::InitTargets();

vector<Target> Target::InitTargets() {
    vector<Target> ret;
    ret.push_back(kNOT_A_TARGET);
    ret.push_back(kLEFT_TILT_TARGET);
    ret.push_back(kRIGHT_TILT_TARGET);
    return ret;
}

const Target Target::from(vision::TargetID id) {
    switch (id) {
        case kLeftTilt:
            return kLEFT_TILT_TARGET;
        case kRightTilt:
            return kRIGHT_TILT_TARGET;
        case kNotATarget:
        default:
            return kNOT_A_TARGET;
    }
}

std::ostream& operator<<(ostream &out, const TargetID &c) {
    out << Target::from(c).getName();
    return out;
}