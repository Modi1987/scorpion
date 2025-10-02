#ifndef LIMB_IK_INTERFACE_
#define LIMB_IK_INTERFACE_

namespace penta_pod::kin::limb_kin_chain {

class LimbIKInterface {
public:
    virtual ~LimbIKInterface() = default;  // ensure proper cleanup

    virtual void init(const int n,
                      const std::vector<double> &a,
                      const std::vector<double> &d,
                      const std::vector<double> &alfa,
                      const std::vector<double> &eef_trans,
                      const std::vector<double> &q_max,
                      const std::vector<double> &q_min) = 0;

    virtual std::vector<double> get_ik(const double &x,
                                       const double &y,
                                       const double &z,
                                       const std::vector<double> &q0) = 0;

    virtual std::string solver_type() = 0;
};


}

#endif // LIMB_IK_INTERFACE_