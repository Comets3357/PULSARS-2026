#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>

class ClimbSubsystem : public frc2::SubsystemBase {
 public:
  ClimbSubsystem();

  void Periodic() override;
  void SetSpeed(double speed);

 private:
  rev::spark::SparkMax m_climbMotor{12, rev::spark::SparkMax::MotorType::kBrushless};
};