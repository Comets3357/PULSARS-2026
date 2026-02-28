#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>

class IndexSubsystem : public frc2::SubsystemBase {
 public:
  IndexSubsystem();

  void Periodic() override;
  void SetSpeed(double speed);

 private:
  rev::spark::SparkMax m_indexMotor{11, rev::spark::SparkMax::MotorType::kBrushless};
};