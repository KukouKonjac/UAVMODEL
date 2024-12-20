#include "UAV_Hover.h"

UAV_Hover::UAV_Hover(const Eigen::Vector3d& pos0, const Eigen::Vector3d& vel0, const Eigen::Vector3d& angle0,
                     const Eigen::Vector3d& omega0_inertial, const Eigen::Vector3d& omega0_body,
                     const Eigen::Vector3d& target_pos)
    : UAV() {
    pos = pos0;
    vel = vel0;
    angle = angle0;
    omega_inertial = omega0_inertial;
    omega_body = omega0_body;
    this->target_pos = target_pos;
    error_pos = target_pos - pos;
}

bool UAV_Hover::is_success() const { return error_pos.norm() < 0.2 && vel.norm() < 0.03; }

bool UAV_Hover::is_Terminal() {
    if (is_success()) {
        std::cout << "Very good!!!" << std::endl;
        terminal_flag = 3;
        is_terminal = true;
        return true;
    }
    return is_out();
}

void UAV_Hover::step_update(const Eigen::VectorXd& action) {
    rk44(action);
    error_pos = target_pos - pos;
    is_terminal = is_Terminal();
}
