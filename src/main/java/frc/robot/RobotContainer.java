// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.vision.Vision;

public class RobotContainer {
  private DriveSubsystem driveSubsystem = new DriveSubsystem();
  private Vision vision = new Vision();
  private CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {
    setDefaultCommands();
    configureBindings();
    setupDashboard();
  }

  public void periodic() {
    vision.update(driveSubsystem);
  }

  /** Sets default commands on all relevant subsystems. */
  private void setDefaultCommands() {
    driveSubsystem.setDefaultCommand(
      Commands.run(() -> {
        // note that controller joystick axes are different from robot axes
        // because the robot follows the NWU coordinate system
        driveSubsystem.driveCartesian(
          -controller.getLeftY(),
          controller.getLeftX(),
          controller.getRightX(),
          true
        );
      }, driveSubsystem)
    );
  }

  private void configureBindings() {
    PathConstraints constraints =
      new PathConstraints(
        Units.FeetPerSecond.of(14.5),
        Units.FeetPerSecondPerSecond.of(30),
        Units.RadiansPerSecond.of(2 * Math.PI), 
        Units.RadiansPerSecondPerSecond.of(2 * Math.PI)
      );

    try {
      if (Robot.isSimulation()) {
        controller.a().onTrue(driveSubsystem.temporarilyDisableMotorSafetyCommand(
            AutoBuilder.pathfindThenFollowPath(
              PathPlannerPath.fromPathFile("Human Player"),
              constraints
            ).andThen(
              AutoBuilder.followPath(PathPlannerPath.fromPathFile("Align Human Player"))
            )
        ));
      }
    } catch (Exception e) {
      DriverStation.reportWarning("path file: 'Human Player' cannot be found. Doing nothing.", false);
    }
  }

  private void setupDashboard() {
    SmartDashboard.putData(driveSubsystem);
  }

  public Command getAutonomousCommand() {
    return Robot.isReal() ? Commands.none() : new PathPlannerAuto("Test Auto");
  }
}
