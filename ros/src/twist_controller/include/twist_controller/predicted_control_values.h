#ifndef PCV__H
#define PCV__H

namespace DBWNODE_NS {

class PredictedControlValues {
    public:
        PredictedControlValues();
        PredictedControlValues(const float throttle, const float brake, const float steer);
        ~PredictedControlValues();
        float throttle();
        float brake();
        float steer();

    private:
        float throttle_;
        float brake_;
        float steer_;
};
}

#endif
