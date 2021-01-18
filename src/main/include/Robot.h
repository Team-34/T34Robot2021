#pragma once

#include <string>
#include <memory>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

#include "subsystems/SwerveDrive.h"
#include "utils/T34_XboxController.h"
#include "utils/RampLimiter.h"

class Robot : public frc::TimedRobot 
{
public:
    Robot();
    ~Robot();
    void RobotInit() override;
    void RobotPeriodic() override;
    void AutonomousInit() override;
    void AutonomousPeriodic() override;
    void TeleopInit() override;
    void TeleopPeriodic() override;
    void TestPeriodic() override;

private:
    frc::SendableChooser<std::string> m_chooser;
    const std::string kAutoNameDefault = "Default";
    const std::string kAutoNameCustom = "My Auto";
    std::string m_autoSelected;

    SwerveDrive * m_swerve_drive;
    T34_XboxController * m_driver_ctrl;
    T34_XboxController * m_mech_ctrl;

    bool m_sheild_wall_on;
    bool m_ramp_limiter_on;
    RampLimiter m_x_limiter;
    RampLimiter m_y_limiter;
    RampLimiter m_r_limiter;    
};
