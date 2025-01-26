package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;

public class Constants {
    private Constants() { throw new RuntimeException("<Constants> Don't initialize me"); }

    public static class MotorControllers {
        private MotorControllers() {
            throw new RuntimeException("<MotorControllers> Don't initialize me");
        }

        // [Default Configs]
        private static final SparkBaseConfig kDefaultConfig =
            new SparkMaxConfig()
                // .closedLoopRampRate(0)
                .voltageCompensation(12)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(80);
        public static final SparkBaseConfig kDefaultNeoConfig =
            new SparkMaxConfig().apply(kDefaultConfig);
        public static final SparkBaseConfig kDefaultNeo550Config =
            new SparkMaxConfig()
                .apply(kDefaultConfig)
                .smartCurrentLimit(20); // small neos need a lower limit

        // [Drivetrain]
        public static final int kFrontLeft  = 1; // placeholder
        public static final int kFrontRight = 2; // placeholder
        public static final int kRearLeft   = 3; // placeholder
        public static final int kRearRight  = 4; // placeholder
    } // ~~~ end MotorControllers ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    public static class DrivetrainConstants {
        private DrivetrainConstants() {
            throw new RuntimeException("<DrivetrainConstants> Don't initialize me");
        }

        // --- Hardware info ----------------------------------------------------------------------
        // [Dimensions]
        public static final Distance kWheelDiameter = Inches.of(6); // placeholder (taken from CAD)
        public static final double kGearRatio = 10; // placeholder
        // horizontal distance from middle to a wheel in meters
        public static final double kDriveTrainBaseX = .212725; // placeholder (taken from CAD)
        // vertical distance from middle to a wheel in meters
        public static final double kDrivetrainBaseY = .2667; // placeholder (taken from CAD)
        public static final Translation2d[] kWheelOffsets =
            new Translation2d[] {
                new Translation2d(kDriveTrainBaseX, kDrivetrainBaseY),
                new Translation2d(kDriveTrainBaseX, -kDrivetrainBaseY),
                new Translation2d(-kDriveTrainBaseX, kDrivetrainBaseY),
                new Translation2d(-kDriveTrainBaseX, -kDrivetrainBaseY),
            };
        // [Physics stuff]
        public static final Mass kMass = Kilograms.of(34); // placeholder (taken from CAD)
        public static final MomentOfInertia kMomentOfInertia = KilogramSquareMeters.of(3.35); // placeholder (taken from CAD)
        // coefficient of friction between the robot's wheel and the floor (carpet at competition)
        // found here: https://www.chiefdelphi.com/t/friction-coefficients-for-omni-wheels-and-mecanum-wheels-from-andymark/74918
        // will probably not have time to figure this out for our particular robot
        public static final double kWheelCoefficientOfFriction = .7;
        public static final LinearVelocity kMaxSpeed = FeetPerSecond.of(14.5); // placeholder
        public static final MecanumDriveKinematics kDriveKinematics =
            new MecanumDriveKinematics(
                kWheelOffsets[0],
                kWheelOffsets[1],
                kWheelOffsets[2],
                kWheelOffsets[3]
            );
        // [Motor Stuff]
        public static final Current kMaxCurrent = Amps.of(50);
        public static final IdleMode kDefaultIdleMode = IdleMode.kBrake;

        // --- PID --------------------------------------------------------------------------------
        // [Pathplanning]
        public static final double kTranslationalXP = 5; // placeholder
        public static final double kTranslationalYP = 5; // placeholder
        public static final double kRotationalP = 5; // placeholder
        // [Motors]
        public static final double kP = 2.0269; // placeholder
        public static final double kD = 0; // placeholder

        // --- Pathplanner ------------------------------------------------------------------------
        public static final RobotConfig kRobotConfig = 
            new RobotConfig(
                kMass,
                kMomentOfInertia,
                new ModuleConfig(
                    kWheelDiameter.times(.5),
                    kMaxSpeed,
                    kWheelCoefficientOfFriction,
                    DCMotor.getNEO(1).withReduction(kGearRatio),
                    kMaxCurrent,
                    1
                ),
                kWheelOffsets
            );
        public static final PPHolonomicDriveController kDriveController =
            new PPHolonomicDriveController(
                new PIDConstants(kTranslationalXP),
                new PIDConstants(kRotationalP)
            );
    } // ~~~ end DriveConstants ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}
