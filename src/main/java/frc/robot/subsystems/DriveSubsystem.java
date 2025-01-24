package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class DriveSubsystem extends SubsystemBase {
    private SparkMax flMotor = new SparkMax(1, MotorType.kBrushless);
    private SparkMax frMotor = new SparkMax(2, MotorType.kBrushless);
    private SparkMax rlMotor = new SparkMax(3, MotorType.kBrushless);
    private SparkMax rrMotor = new SparkMax(4, MotorType.kBrushless);
    // private MecanumDrive drivetrain = new MecanumDrive(flMotor, rlMotor, frMotor, rrMotor);
    private AHRS gyro;
    private final double kGearRatio = 10;
    private final double kWheelDiameterMeters = Units.inchesToMeters(6);

    public DriveSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();

        flMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rlMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        // inverse right side of the drivetrain
        config.inverted(true);
        frMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rrMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        // attempt to instantiate the gyroscope
        try {
            gyro = new AHRS(SPI.Port.kMXP);
        } catch (RuntimeException e) {
            DriverStation.reportError("Error instantiating navX-MXP:  " + e.getMessage(), true);
        }
    }

    // --- Public Methods -------------------------------------------------------------------------

    /**
     * Drive method for Mecanum platform.
     *
     * Angles are measured counterclockwise from the positive X axis. The robot's speed is
     * independent of its angle or rotation rate.
     *
     * @param xSpeed The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
     * @param ySpeed The robot's speed along the Y axis [-1.0..1.0]. Left is positive.
     * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Counterclockwise is
     *     positive.
     * @param isFieldOriented Determines whether to drive the robot relative to the field, or to itself.
     */
    public void driveCartesian(double xSpeed, double ySpeed, double zRotation, boolean isFieldOriented) {
        Rotation2d heading = new Rotation2d();
        if (isFieldOriented && gyro != null) {
            heading = gyro.getRotation2d().unaryMinus();
        }
        // drivetrain.driveCartesian(xSpeed, ySpeed, zRotation, heading);
    }

    public Command stop() {
        return run(() -> {
            flMotor.set(0);
            frMotor.set(0);
            rlMotor.set(0);
            rrMotor.set(0);
        });
    }

    // --- Private Methods ------------------------------------------------------------------------

    private MecanumDriveWheelPositions getWheelPositions() {
        // converting rotations to meters
        return new MecanumDriveWheelPositions(
            flMotor.getEncoder().getPosition() * Math.PI * kWheelDiameterMeters / kGearRatio,
            frMotor.getEncoder().getPosition() * Math.PI * kWheelDiameterMeters / kGearRatio,
            rlMotor.getEncoder().getPosition() * Math.PI * kWheelDiameterMeters / kGearRatio,
            rrMotor.getEncoder().getPosition() * Math.PI * kWheelDiameterMeters / kGearRatio
        );
    }

    private MecanumDriveWheelSpeeds getWheelSpeeds() {
        // converting rpm to mps
        return new MecanumDriveWheelSpeeds(
            flMotor.getEncoder().getVelocity() * Math.PI / 60 * kWheelDiameterMeters / kGearRatio,
            frMotor.getEncoder().getVelocity() * Math.PI / 60 * kWheelDiameterMeters / kGearRatio,
            rlMotor.getEncoder().getVelocity() * Math.PI / 60 * kWheelDiameterMeters / kGearRatio,
            rrMotor.getEncoder().getVelocity() * Math.PI / 60 * kWheelDiameterMeters / kGearRatio
        );
    }

    // --- SysId ----------------------------------------------------------------------------------

    public SysIdRoutine getRoutine() {
        return new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Second).of(1),
                Volts.of(7),
                Seconds.of(5)
            ),
            new SysIdRoutine.Mechanism(
                volts -> {
                    flMotor.setVoltage(volts.in(Volts));
                    frMotor.setVoltage(volts.in(Volts));
                    rlMotor.setVoltage(volts.in(Volts));
                    rrMotor.setVoltage(volts.in(Volts));
                },
                log -> {
                    var positions = getWheelPositions();
                    var speeds = getWheelSpeeds();
                    log.motor("frontLeft")
                       .voltage(Volts.of(flMotor.getAppliedOutput() * RobotController.getBatteryVoltage()))
                       .linearPosition(Meters.of(positions.frontLeftMeters))
                       .linearVelocity(MetersPerSecond.of(speeds.frontLeftMetersPerSecond));
                    log.motor("frontRight")
                       .voltage(Volts.of(frMotor.getAppliedOutput() * RobotController.getBatteryVoltage()))
                       .linearPosition(Meters.of(positions.frontRightMeters))
                       .linearVelocity(MetersPerSecond.of(speeds.frontRightMetersPerSecond));
                    log.motor("rearLeft")
                       .voltage(Volts.of(rlMotor.getAppliedOutput() * RobotController.getBatteryVoltage()))
                       .linearPosition(Meters.of(positions.rearLeftMeters))
                       .linearVelocity(MetersPerSecond.of(speeds.rearLeftMetersPerSecond));
                    log.motor("rearRight")
                       .voltage(Volts.of(rrMotor.getAppliedOutput() * RobotController.getBatteryVoltage()))
                       .linearPosition(Meters.of(positions.rearRightMeters))
                       .linearVelocity(MetersPerSecond.of(speeds.rearRightMetersPerSecond));
                },
                this
            )
        );
    }

    // --- SubsystemBase --------------------------------------------------------------------------

    @Override
    public void periodic() {}

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Yaw", () -> gyro.getYaw(), null);
    }
}
