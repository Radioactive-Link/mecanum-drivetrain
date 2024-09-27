// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
  private DriveSubsystem driveSubsystem = new DriveSubsystem();
  private CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {
    setDefaultCommands();
    configureBindings();
  }

  /** Sets default commands on all relevant subsystems. */
  private void setDefaultCommands() {
    driveSubsystem.setDefaultCommand(
      Commands.run(() -> {
        // note that controller joystick axes are different from robot axes
        // because the robot follows the NWU coordinate system
        driveSubsystem.driveCartesian(
          controller.getLeftY(),
          -controller.getLeftX(),
          -controller.getRightX(),
          true
        );
      }, driveSubsystem)
    );
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
