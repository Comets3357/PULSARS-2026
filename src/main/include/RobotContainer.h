// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc/smartdashboard/SendableChooser.h>

#include "Subsystems/DriveSubsystem.h"
#include "Subsystems/IndexSubsystem.h"
#include "Subsystems/ShooterSubsystem.h"
#include "Subsystems/ClimbSubsystem.h"


class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

 private:
  void ConfigureBindings();
  void ConfigureAutos();
  frc2::CommandPtr PotatoAuton();
  frc2::CommandPtr ShootFromRangeAuton();

  DriveSubsystem drive{};
  frc2::CommandXboxController controller{0};
  IndexSubsystem indexSubsystem{};
  ShooterSubsystem shooterSubsystem{};
  ClimbSubsystem climbSubsystem{};

  frc::SendableChooser<frc2::Command*> autoChooser;

  frc2::CommandPtr potatoAuto = PotatoAuton();
  frc2::CommandPtr rangeAuto = ShootFromRangeAuton();

};
