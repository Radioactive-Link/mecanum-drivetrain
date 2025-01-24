// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
  private DriveSubsystem driveSubsystem = new DriveSubsystem();
  private CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {
    setDefaultCommands();
    configureBindings();
    setupDashboard();
  }

  /** Sets default commands on all relevant subsystems. */
  private void setDefaultCommands() {
    driveSubsystem.setDefaultCommand(
      driveSubsystem.stop()
    );
  }

  private void configureBindings() {
    // [SysId Bindings]
    var routine = driveSubsystem.getRoutine();
    // button must be held down to complete routine
    controller.y().whileTrue(routine.quasistatic(Direction.kForward));
    controller.a().whileTrue(routine.quasistatic(Direction.kReverse));
    // dpad up
    controller.pov(0).whileTrue(routine.dynamic(Direction.kForward));
    // dpad down
    controller.pov(180).whileTrue(routine.dynamic(Direction.kReverse));
  }

  private void setupDashboard() {
    SmartDashboard.putData(driveSubsystem);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
