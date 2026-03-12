// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/MathUtil.h>
#include <frc2/command/Commands.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/WaitUntilCommand.h>
#include <frc/DriverStation.h>
#include "Constants.h"

RobotContainer::RobotContainer() {
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  drive.SetDefaultCommand(frc2::RunCommand(
    [this] {

    double leftY = frc::ApplyDeadband(controller.GetLeftY(), OIConstants::kDriveDeadband);
    double leftX = frc::ApplyDeadband(controller.GetLeftX(), OIConstants::kDriveDeadband);
    double rightX = frc::ApplyDeadband(controller.GetRightX(), OIConstants::kDriveDeadband);

    std::optional<frc::DriverStation::Alliance> alliance = frc::DriverStation::GetAlliance();
    if (alliance.value() == frc::DriverStation::Alliance::kRed) {
      leftY = -leftY;
      leftX = -leftX;
      rightX = -rightX;
    }

    drive.Drive(
      -units::meters_per_second_t{leftY},
      -units::meters_per_second_t{leftX},
      -units::radians_per_second_t{rightX},
      true);
    },
    {&drive}));
  // Ways to make the life of the drivers easier:
  // Indexer always moving in the background but slowly. When shooting, it moves faster.
  // Always intaking when not shooting.
  // Max limit on climb extension.

  controller.Start().OnTrue(frc2::cmd::RunOnce([this] () { drive.ZeroHeading(); }, {}));

  //Putting in hopper
  // controller.B().OnTrue(frc2::cmd::RunOnce([this] () { shooterSubsystem.SetSpeed(0.5);}, {&shooterSubsystem}));
  // controller.B().OnFalse(frc2::cmd::RunOnce([this] () { shooterSubsystem.SetSpeed(0);}, {&shooterSubsystem}));
  // controller.B().OnTrue(frc2::cmd::RunOnce([this] () { indexSubsystem.SetSpeed(-0.75);}, {&indexSubsystem}));
  // controller.B().OnFalse(frc2::cmd::RunOnce([this] () { indexSubsystem.SetSpeed(0);}, {&indexSubsystem}));

  //Shooting
  controller.RightBumper().OnFalse(frc2::cmd::RunOnce([this] () { shooterSubsystem.SetSpeed(0.5);}, {&shooterSubsystem}));
  controller.RightBumper().OnFalse(frc2::cmd::RunOnce([this] () { indexSubsystem.SetSpeed(-0.75);}, {&indexSubsystem}));
  
  controller.RightBumper().OnTrue(frc2::cmd::RunOnce([this] () { shooterSubsystem.SetSpeed(1); indexSubsystem.SetSpeed(0);}, {&shooterSubsystem, &indexSubsystem})
  .AlongWith(frc2::WaitUntilCommand([this]()->bool { return shooterSubsystem.GetVelocity() > 4000;}).ToPtr())
  .AndThen(frc2::cmd::RunOnce([this] () {indexSubsystem.SetSpeed(1);}, {&indexSubsystem})));
  
  //Climbing
  //controller.RightBumper().OnTrue(frc2::cmd::RunOnce([this] () { climbSubsystem.SetSpeed(0.3);}, {&climbSubsystem}));
  //controller.LeftBumper().OnTrue(frc2::cmd::RunOnce([this] () { climbSubsystem.SetSpeed(-0.3);}, {&climbSubsystem}));
  //controller.RightBumper().OnFalse(frc2::cmd::RunOnce([this] () { climbSubsystem.SetSpeed(0);}, {&climbSubsystem}));
  // controller.LeftBumper().OnFalse(frc2::cmd::RunOnce([this] () { climbSubsystem.SetSpeed(0);}, {&climbSubsystem}));

  //Auto go to range
  controller.A().WhileTrue(frc2::cmd::Run([this] () {
    std::optional<frc::DriverStation::Alliance> alliance = frc::DriverStation::GetAlliance();
    if (alliance.has_value()){
        frc::Translation2d hub;
        if (alliance.value() == frc::DriverStation::Alliance::kBlue){
            hub = blueHubLocation;
        }else{
            hub = redHubLocation;
        }
        drive.GoToRange(2_m, hub);
        //todo: tune range
    }
  }, {&drive}));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}

