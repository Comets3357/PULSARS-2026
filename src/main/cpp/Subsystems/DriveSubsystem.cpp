// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Subsystems/DriveSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <hal/FRCUsageReporting.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <frc/DriverStation.h>

#include "Constants.h"

using namespace DriveConstants;

DriveSubsystem::DriveSubsystem()
    : m_frontLeft{kFrontLeftDrivingCanId, kFrontLeftTurningCanId,
                  kFrontLeftChassisAngularOffset},
      m_rearLeft{kRearLeftDrivingCanId, kRearLeftTurningCanId,
                 kRearLeftChassisAngularOffset},
      m_frontRight{kFrontRightDrivingCanId, kFrontRightTurningCanId,
                   kFrontRightChassisAngularOffset},
      m_rearRight{kRearRightDrivingCanId, kRearRightTurningCanId,
                  kRearRightChassisAngularOffset},
      m_odometry{kDriveKinematics,
                 frc::Rotation2d(GetHeading()),
                 {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                  m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
                 frc::Pose2d{}} {
  // Usage reporting for MAXSwerve template
  HAL_Report(HALUsageReporting::kResourceType_RobotDrive,
             HALUsageReporting::kRobotDriveSwerve_MaxSwerve);
}

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  m_odometry.Update(frc::Rotation2d(GetHeading()),
                    {m_frontLeft.GetPosition(), m_rearLeft.GetPosition(),
                     m_frontRight.GetPosition(), m_rearRight.GetPosition()});
  
  std::optional<std::pair<frc::Pose2d, units::second_t>> visionPose = m_visionSubsystem.GetLatestPose();
  if (visionPose.has_value()){
    m_odometry.AddVisionMeasurement(visionPose.value().first, visionPose.value().second);
  }

  m_field.SetRobotPose(m_odometry.GetEstimatedPosition());
  frc::SmartDashboard::PutData("RobotPose", &m_field);

  frc::SmartDashboard::PutNumber("GyroAngle", GetHeading().value());
}

void DriveSubsystem::TurnToAngle(units::meters_per_second_t xSpeed,
                                units::meters_per_second_t ySpeed,
                                units::degree_t targetAngle) {
  units::degree_t currentAngle = GetHeading();
  const auto kp_value = 1.0 / 1.0_s; // TODO: tune this value
  units::degree_t angleError = targetAngle - currentAngle;
  units::degrees_per_second_t control = angleError * kp_value;


  if (std::abs(angleError.value()) > 180.0)
  {

    control = -control;
  }

  Drive(xSpeed, ySpeed, control, true);
}

void DriveSubsystem::GoToPosition(units::meter_t targetRange, frc::Translation2d target) {
  frc::Pose2d currentPos = GetPose();
  frc::Translation2d targetVector = target - currentPos.Translation();
  units::meter_t currentRange = targetVector.Norm();
  const auto kp_value = 1.0 / 2.0_s; // TODO: tune this value

  TurnToTarget(targetVector.X() * kp_value, targetVector.Y() * kp_value, target);
}

void DriveSubsystem::GoToRange(units::meter_t targetRange, frc::Translation2d target) {
  frc::Pose2d currentPos = GetPose();
  frc::Translation2d targetVector = target - currentPos.Translation();
  units::meter_t currentRange = targetVector.Norm();
  const auto kp_value = 1.0 / 2.0_s; // TODO: tune this value

  frc::Translation2d controlVector = targetVector + (targetVector.RotateBy(frc::Rotation2d(units::degree_t{180})) / targetVector.Norm().value() * targetRange.value()); 

  TurnToTarget(controlVector.X() * kp_value, controlVector.Y() * kp_value, target);
}

void DriveSubsystem::TurnToTarget(units::meters_per_second_t xSpeed,
                                units::meters_per_second_t ySpeed,
                                frc::Translation2d target){
  frc::Pose2d currentPos = GetPose();
  frc::Translation2d targetVector = target - currentPos.Translation();
  units::degree_t targetAngle = targetVector.Angle().RotateBy(180_deg).Degrees();

  TurnToAngle(xSpeed, ySpeed, targetAngle);
}


void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot,
                           bool fieldRelative) {
  // Convert the commanded speeds into the correct units for the drivetrain
  units::meters_per_second_t xSpeedDelivered =
      xSpeed.value() * DriveConstants::kMaxSpeed;
  units::meters_per_second_t ySpeedDelivered =
      ySpeed.value() * DriveConstants::kMaxSpeed;
  units::radians_per_second_t rotDelivered =
      rot.value() * DriveConstants::kMaxAngularSpeed;

  auto states = kDriveKinematics.ToSwerveModuleStates(
      fieldRelative
          ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                xSpeedDelivered, ySpeedDelivered, rotDelivered,
                frc::Rotation2d(GetHeading()))
          : frc::ChassisSpeeds{xSpeedDelivered, ySpeedDelivered, rotDelivered});

  kDriveKinematics.DesaturateWheelSpeeds(&states, DriveConstants::kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_rearLeft.SetDesiredState(bl);
  m_rearRight.SetDesiredState(br);
}

void DriveSubsystem::SetX() {
  m_frontLeft.SetDesiredState(
      frc::SwerveModuleState{0_mps, frc::Rotation2d{45_deg}});
  m_frontRight.SetDesiredState(
      frc::SwerveModuleState{0_mps, frc::Rotation2d{-45_deg}});
  m_rearLeft.SetDesiredState(
      frc::SwerveModuleState{0_mps, frc::Rotation2d{-45_deg}});
  m_rearRight.SetDesiredState(
      frc::SwerveModuleState{0_mps, frc::Rotation2d{45_deg}});
}

void DriveSubsystem::SetModuleStates(
    wpi::array<frc::SwerveModuleState, 4> desiredStates) {
  kDriveKinematics.DesaturateWheelSpeeds(&desiredStates,
                                         DriveConstants::kMaxSpeed);
  m_frontLeft.SetDesiredState(desiredStates[0]);
  m_frontRight.SetDesiredState(desiredStates[1]);
  m_rearLeft.SetDesiredState(desiredStates[2]);
  m_rearRight.SetDesiredState(desiredStates[3]);
}

void DriveSubsystem::ResetEncoders() {
  m_frontLeft.ResetEncoders();
  m_rearLeft.ResetEncoders();
  m_frontRight.ResetEncoders();
  m_rearRight.ResetEncoders();
}

units::degree_t DriveSubsystem::GetHeading() {
  units::degree_t current_yaw = m_gyro.GetYaw() + offset;
  while (current_yaw > 180_deg)
  {
    current_yaw = current_yaw - 360_deg;
  }
  while (current_yaw < -180_deg)
  {
    current_yaw = current_yaw + 360_deg;
  }

  return current_yaw;
}

void DriveSubsystem::ZeroHeading() { 
  if (frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kRed) {
    offset = -m_gyro.GetYaw() + 180_deg;
  } else {
    offset = -m_gyro.GetYaw();
  }
}

double DriveSubsystem::GetTurnRate() {
  return -m_gyro.GetAngularVelocityYaw().convert<units::degrees_per_second>().value();
}

frc::Pose2d DriveSubsystem::GetPose() { return m_odometry.GetEstimatedPosition(); }

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
  m_odometry.ResetPosition(
      GetHeading(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
      pose);
}
