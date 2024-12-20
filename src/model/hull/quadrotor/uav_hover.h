#ifndef UAV_HOVER_H
#define UAV_HOVER_H

#include "UAV.h"

class UAV_Hover : public UAV {
  public:
    UAV_Hover(const Eigen::Vector3d& pos0 = Eigen::Vector3d::Zero(),
              const Eigen::Vector3d& vel0 = Eigen::Vector3d::Zero(),
              const Eigen::Vector3d& angle0 = Eigen::Vector3d::Zero(),
              const Eigen::Vector3d& omega0_inertial = Eigen::Vector3d::Zero(),
              const Eigen::Vector3d& omega0_body = Eigen::Vector3d::Zero(),
              const Eigen::Vector3d& target_pos = Eigen::Vector3d::Zero());

    bool is_success() const;
    bool is_Terminal();
    void step_update(const Eigen::VectorXd& action);

  private:
    Eigen::Vector3d target_pos, error_pos;
};

#endif // UAV_HOVER_H
