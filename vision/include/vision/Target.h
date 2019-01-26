//
// Created by ryan on 1/5/17.
//

#ifndef VISION_TARGET_H
#define VISION_TARGET_H

#include <ostream>
#include <string>
#include <vector>
#include <opencv2/core.hpp>

namespace vision {
    enum TargetID {
        kNotATarget,
        kLeftTilt,
        kRightTilt,
    };

    class Target {
    private:
        static const std::vector<Target> TARGETS;
        static std::vector<Target> InitTargets();

    public:
        static const Target kNOT_A_TARGET;
        static const Target kLEFT_TILT_TARGET;
        static const Target kRIGHT_TILT_TARGET;
        static const std::vector<Target> GetTargets();
        static const Target from(TargetID id);

    private:
        TargetID m_id;
        double m_aspectRatio;
        std::string m_name;

        Target(TargetID id, double aspectRatio, std::string name) {
            m_id = id;
            m_aspectRatio = aspectRatio;
            m_name = name;
        }

    public:
        TargetID getID() const { return m_id; }
        double aspectRatio() const { return m_aspectRatio; }
        std::string getName() const { return m_name; }

        inline bool operator==(Target &other) {
            return this->getID() == other.getID();
        }
        inline bool operator==(const Target &other) {
            return this->getID() == other.getID();
        }
    };
}

std::ostream& operator<<(std::ostream &out, const vision::TargetID &c);

#endif //VISION_TARGET_H
