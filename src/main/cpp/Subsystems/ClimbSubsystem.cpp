#include "Subsystems/ClimbSubsystem.h"

ClimbSubsystem::ClimbSubsystem() {
  rev::spark::SparkBaseConfig config;
  config.SmartCurrentLimit(40);
  config.Inverted(true);
  config.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kCoast);
  m_climbMotor.Configure(config, rev::ResetMode::kResetSafeParameters,
                          rev::PersistMode::kPersistParameters);
}

void ClimbSubsystem::Periodic() {
}

void ClimbSubsystem::SetSpeed(double speed) {
  m_climbMotor.Set(speed);
}