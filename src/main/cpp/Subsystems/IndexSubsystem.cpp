#include "Subsystems/IndexSubsystem.h"

IndexSubsystem::IndexSubsystem() {
  rev::spark::SparkBaseConfig config;
  config.SmartCurrentLimit(40);
  config.Inverted(true);
  config.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kCoast);
  m_indexMotor.Configure(config, rev::ResetMode::kResetSafeParameters,
                          rev::PersistMode::kPersistParameters);
}

void IndexSubsystem::Periodic() {
}

void IndexSubsystem::SetSpeed(double speed) {
  m_indexMotor.Set(speed);
}