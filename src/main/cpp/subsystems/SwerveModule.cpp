#include "subsystems/SwerveMath.h"
#include "subsystems/SwerveModule.h"

SwerveModule::SwerveModule(const int drive_id, const int steer_id, int invert)
{
    m_drive = new WPI_TalonFX(drive_id);
    m_steer = new WPI_TalonFX(steer_id);

    m_steer->SetNeutralMode(NeutralMode::Coast);
    m_steer->Set(ControlMode::Position, 0);
    m_steer->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 0);
    m_steer->SetInverted(false);
    m_steer->Config_kD(0, 0.01, 0);

    m_drive->SetNeutralMode(NeutralMode::Brake);
    m_drive_brake_on = true;
    
    m_default_invert = m_invert = invert;
}

SwerveModule::~SwerveModule()
{
    if (m_drive)
        delete m_drive;
        
    if (m_steer)
        delete m_steer;
}

void SwerveModule::SetDriveBrake(bool on)
{
    if (on)
    {
        m_drive->SetNeutralMode(NeutralMode::Brake);    
        m_drive_brake_on = true;
    }
    else
    {
        m_drive->SetNeutralMode(NeutralMode::Coast);    
        m_drive_brake_on = false;
    }
}

void SwerveModule::ToggleDriveBrake()
{
    SetDriveBrake(!m_drive_brake_on);
}

void SwerveModule::SetDriveSpeed(const double & speed)
{
    m_drive->Set(ControlMode::PercentOutput, m_invert * speed);
}

void SwerveModule::SetSteerPosition(const double & position, double offset)
{
    double current_position = m_steer->GetSensorCollection().GetIntegratedSensorPosition();
 //   double normalized_position = current_position / FULL_UNITS;fmod(current_position, FULL_UNITS);
	double set_point = ((position + offset) / 360.0) * FULL_UNITS;

/*
    if(fabs(set_point - current_position) > 18432)
    {
        set_point = set_point + 18432;
        m_invert = -1.0;
    }
*/
    if (set_point > 18432)
    {
        set_point = set_point - 18432;
        m_invert *= -1.0;
    }
    else
    {
        m_invert = m_default_invert;
    }
    
    
    m_steer->Set(ControlMode::Position, set_point);

 
}

void SwerveModule::ZeroWheel()
{
     m_steer->GetSensorCollection().SetIntegratedSensorPosition(0.0);
}