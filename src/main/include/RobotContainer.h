// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>


#include "Subsystems/IndexSubsystem.h"
#include "Subsystems/DriveSubsystem.h"
#include "Subsystems/ShooterSubsystem.h"
#include "Subsystems/ClimbSubsystem.h"

class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

 private:
  void ConfigureBindings();

  DriveSubsystem drive{};
  frc2::CommandXboxController controller{0};
  IndexSubsystem indexSubsystem{};
  ShooterSubsystem shooterSubsystem{};
  ClimbSubsystem climbSubsystem{};
};
