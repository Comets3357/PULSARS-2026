#include "Subsystems/ShooterSubsystem.h"

ShooterSubsystem::ShooterSubsystem() {
    rev::spark::SparkBaseConfig leader_config;
    rev::spark::SparkBaseConfig follower_config;
  leader_config.SmartCurrentLimit(40);
  leader_config.Inverted(false);
  leader_config.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kCoast);
  m_shooterMotor.Configure(leader_config, rev::spark::SparkBase::ResetMode::kResetSafeParameters,
                          rev::spark::SparkBase::PersistMode::kPersistParameters);

  follower_config.SmartCurrentLimit(40);
  follower_config.Inverted(true);
  follower_config.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kCoast);
  follower_config.Follow(m_shooterMotor, true);
  m_shooterFollower.Configure(follower_config, rev::spark::SparkBase::ResetMode::kResetSafeParameters,
                          rev::spark::SparkBase::PersistMode::kPersistParameters);
}

void ShooterSubsystem::Periodic() {
    flyWheelSpeed.Set(m_shooterEncoder.GetVelocity());
}

void ShooterSubsystem::SetSpeed(double speed) {
    m_shooterMotor.Set(speed);
}

double ShooterSubsystem::GetVelocity(){

    return m_shooterEncoder.GetVelocity();
}