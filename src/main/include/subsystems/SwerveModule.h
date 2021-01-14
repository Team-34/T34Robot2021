#pragma once

#include <ctre/Phoenix.h>

class SwerveModule
{
public:
    SwerveModule(const int drive_id, const int steer_id, int invert = 1.0);
    ~SwerveModule();

    void SetDriveBrake(bool on = true);
    void ToggleDriveBrake();    
    
    void SetDriveSpeed(const double & speed);
    void SetSteerPosition(const double & posistion, double offset = 0.0);

    double m_default_invert;
    double m_invert;

    void ZeroWheel();

private:
    WPI_TalonFX * m_drive;
    WPI_TalonFX * m_steer;
    
    
    bool m_drive_brake_on;
};