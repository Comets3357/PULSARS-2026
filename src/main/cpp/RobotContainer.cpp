// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/MathUtil.h>
#include <frc2/command/Commands.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/WaitUntilCommand.h>

RobotContainer::RobotContainer() {
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        drive.Drive(
            -units::meters_per_second_t{frc::ApplyDeadband(
                controller.GetLeftY(), OIConstants::kDriveDeadband)},
            -units::meters_per_second_t{frc::ApplyDeadband(
                controller.GetLeftX(), OIConstants::kDriveDeadband)},
            -units::radians_per_second_t{frc::ApplyDeadband(
                controller.GetRightX(), OIConstants::kDriveDeadband)},
            true);
      },
      {&drive}));
  // Ways to make the life of the drivers easier:
  // Indexer always moving in the background but slowly. When shooting, it moves faster.
  // Always intaking when not shooting.
  // Max limit on climb extension.

  /*
  controller.A().OnTrue(frc2::cmd::RunOnce([this] () { indexSubsystem.SetSpeed(0.5);}, {&indexSubsystem}));
  controller.A().OnFalse(frc2::cmd::RunOnce([this] () { indexSubsystem.SetSpeed(0);}, {&indexSubsystem}));
  controller.B().OnTrue(frc2::cmd::RunOnce([this] () { shooterSubsystem.SetSpeed(0.5);}, {&shooterSubsystem}));
  controller.B().OnFalse(frc2::cmd::RunOnce([this] () { shooterSubsystem.SetSpeed(0);}, {&shooterSubsystem}));
  controller.X().OnTrue(frc2::cmd::RunOnce([this] () { climbSubsystem.SetSpeed(0.5);}, {&climbSubsystem}));
  controller.X().OnFalse(frc2::cmd::RunOnce([this] () { climbSubsystem.SetSpeed(0);}, {&climbSubsystem}));
  */

  //Putting in hopper
  controller.B().OnTrue(frc2::cmd::RunOnce([this] () { shooterSubsystem.SetSpeed(0.5);}, {&shooterSubsystem}));
  controller.B().OnFalse(frc2::cmd::RunOnce([this] () { shooterSubsystem.SetSpeed(0);}, {&shooterSubsystem}));
  controller.B().OnTrue(frc2::cmd::RunOnce([this] () { indexSubsystem.SetSpeed(-0.5);}, {&indexSubsystem}));
  controller.B().OnFalse(frc2::cmd::RunOnce([this] () { indexSubsystem.SetSpeed(0);}, {&indexSubsystem}));

  //Shooting
  controller.A().OnFalse(frc2::cmd::RunOnce([this] () { shooterSubsystem.SetSpeed(0);}, {&shooterSubsystem}));
  controller.A().OnFalse(frc2::cmd::RunOnce([this] () { indexSubsystem.SetSpeed(0);}, {&indexSubsystem}));
  
  controller.A().OnTrue(frc2::cmd::RunOnce([this] () { shooterSubsystem.SetSpeed(1); indexSubsystem.SetSpeed(0);}, {&shooterSubsystem, &indexSubsystem})
  .AlongWith(frc2::WaitUntilCommand([this]()->bool { return shooterSubsystem.GetVelocity() > 3500;}).ToPtr())
  .AndThen(frc2::cmd::RunOnce([this] () {indexSubsystem.SetSpeed(1);}, {&indexSubsystem})));

}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}

