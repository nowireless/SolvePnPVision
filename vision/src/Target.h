//
// Created by ryan on 1/5/17.
//

#ifndef VISION_TARGET_H
#define VISION_TARGET_H

#include <vector>
#include <opencv2/core.hpp>

namespace vision {
    enum TargetID {
        kNotATarget,
        kHorizontal,
        kVertical
    };

    class Target {
    private:
        static const std::vector<Target> TARGETS;
        static std::vector<Target> InitTargets();

    public:
        static const Target kNOT_A_TARGET;
        static const Target kHORIZONTAL_TARGET;
        static const Target kVERTICAL_TARGET;
        static const std::vector<Target> GetTargets();

    private:
        TargetID m_id;
        double m_aspectRatio;


        Target(TargetID id, double aspectRatio) {
            m_id = id;
            m_aspectRatio = aspectRatio;
        }

    public:
        TargetID getID() const { return m_id; }
        double aspectRatio() const { return m_aspectRatio; }

        inline bool operator==(Target &other) {
            return this->getID() == other.getID();
        }
        inline bool operator==(const Target &other) {
            return this->getID() == other.getID();
        }
    };
}

#endif //VISION_TARGET_H
