#pragma once
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTableInstance.h>

class ShooterSubsystem : public frc2::SubsystemBase {
 public:
  ShooterSubsystem();

  void Periodic() override;
  void SetSpeed(double speed);
  double GetVelocity();

 private:
    rev::spark::SparkMax m_shooterMotor{15, rev::spark::SparkMax::MotorType::kBrushless};
    rev::spark::SparkMax m_shooterFollower{16, rev::spark::SparkMax::MotorType::kBrushless};

    rev::spark::SparkRelativeEncoder m_shooterEncoder = m_shooterMotor.GetEncoder();

    nt::DoublePublisher flyWheelSpeed = nt::NetworkTableInstance::GetDefault().GetDoubleTopic("Flywheel setpoint").Publish();
};