package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
    private SparkMax flMotor = new SparkMax(1, MotorType.kBrushless);
    private SparkMax frMotor = new SparkMax(2, MotorType.kBrushless);
    private SparkMax rlMotor = new SparkMax(3, MotorType.kBrushless);
    private SparkMax rrMotor = new SparkMax(4, MotorType.kBrushless);
    private MecanumDrive drivetrain = new MecanumDrive(flMotor, rlMotor, frMotor, rrMotor);
    private AHRS gyro;

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
        drivetrain.driveCartesian(xSpeed, ySpeed, zRotation, heading);
    }

    // --- SubsystemBase --------------------------------------------------------------------------

    @Override
    public void periodic() {}

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Yaw", () -> gyro.getYaw(), null);
    }
}
