package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class DriveSubsystem extends SubsystemBase {
    private SparkMax flMotor = new SparkMax(1, MotorType.kBrushless);
    private SparkMax frMotor = new SparkMax(2, MotorType.kBrushless);
    private SparkMax rlMotor = new SparkMax(3, MotorType.kBrushless);
    private SparkMax rrMotor = new SparkMax(4, MotorType.kBrushless);
    private MecanumDrive drivetrain = new MecanumDrive(flMotor, rlMotor, frMotor, rrMotor);
    private SparkMax[] motors = new SparkMax[] {flMotor, frMotor, rlMotor, rrMotor};
    private MecanumDriveKinematics kinematics =
        // sample values
        new MecanumDriveKinematics(
            new Translation2d(.35, .25), new Translation2d(.35, -.25),
            new Translation2d(-.35, .25), new Translation2d(-.35, -.25)
        );
    private MecanumDrivePoseEstimator poseEstimator;
    private AHRS gyro;

    // telemetry & sim
    private Field2d field = new Field2d();
    private StructPublisher<Pose2d> posePublisher =
        NetworkTableInstance.getDefault().getStructTopic("Robot Pose", Pose2d.struct).publish();
    private Rotation2d simHeading = new Rotation2d();
    // sim motors
    private DCMotor neo = DCMotor.getNEO(1);
    private SparkSim[] sparkSims =
        new SparkSim[] {
            new SparkSim(flMotor, neo),
            new SparkSim(frMotor, neo),
            new SparkSim(rlMotor, neo),
            new SparkSim(rrMotor, neo)
        };
    // used for velocity calculations
    private DCMotorSim[] motorSims =
        new DCMotorSim[] {
            new DCMotorSim(LinearSystemId.createDCMotorSystem(neo, 6.66E-15, 1), neo),
            new DCMotorSim(LinearSystemId.createDCMotorSystem(neo, 6.66E-15, 1), neo),
            new DCMotorSim(LinearSystemId.createDCMotorSystem(neo, 6.66E-15, 1), neo),
            new DCMotorSim(LinearSystemId.createDCMotorSystem(neo, 6.66E-15, 1), neo),
        };
    private SparkRelativeEncoderSim[] encodersSims =
        new SparkRelativeEncoderSim[] {
            new SparkRelativeEncoderSim(flMotor),
            new SparkRelativeEncoderSim(frMotor),
            new SparkRelativeEncoderSim(rlMotor),
            new SparkRelativeEncoderSim(rrMotor)
        };

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

        poseEstimator = new MecanumDrivePoseEstimator(kinematics, gyro.getRotation2d(), getWheelPositions(), new Pose2d());

        // telemetry
        SmartDashboard.putData("Field", field);
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
        if (isFieldOriented) {
            if (!Robot.isReal()) {
                // pi/2 offset is for driving relative to the screen when field image is horizontal
                heading = simHeading.unaryMinus().plus(Rotation2d.fromRadians(Math.PI / 2));
            } else if (gyro != null) {
                heading = gyro.getRotation2d().unaryMinus();
            }
        }
        drivetrain.driveCartesian(xSpeed, ySpeed, zRotation, heading);
    }

    // --- Private Methods ------------------------------------------------------------------------

    private MecanumDriveWheelPositions getWheelPositions() {
        // converting rotations to meters. Wheel diameter is assumed to be 6in
        return new MecanumDriveWheelPositions(
            flMotor.getEncoder().getPosition() * Math.PI * Units.inchesToMeters(6),
            frMotor.getEncoder().getPosition() * Math.PI * Units.inchesToMeters(6),
            rlMotor.getEncoder().getPosition() * Math.PI * Units.inchesToMeters(6),
            rrMotor.getEncoder().getPosition() * Math.PI * Units.inchesToMeters(6)
        );
    }

    private MecanumDriveWheelSpeeds getWheelSpeeds() {
        // converting rpm to mps
        return new MecanumDriveWheelSpeeds(
            flMotor.getEncoder().getVelocity() * Math.PI / 60 * Units.inchesToMeters(6),
            frMotor.getEncoder().getVelocity() * Math.PI / 60 * Units.inchesToMeters(6),
            rlMotor.getEncoder().getVelocity() * Math.PI / 60 * Units.inchesToMeters(6),
            rrMotor.getEncoder().getVelocity() * Math.PI / 60 * Units.inchesToMeters(6)
        );
    }

    // --- SubsystemBase --------------------------------------------------------------------------

    @Override 
    public void periodic() {
        if (!Robot.isReal()) {
            // --- update sims ---
            for (int i = 0; i < motorSims.length; ++i) {
                // calculate sim measurements (mainly velocity) from input voltage
                motorSims[i].setInputVoltage(motors[i].getAppliedOutput());
                motorSims[i].update(0.02);
                // apply it to spark
                sparkSims[i].iterate(motorSims[i].getAngularVelocityRPM(), RoboRioSim.getVInVoltage(), 0.02);
                encodersSims[i].iterate(sparkSims[i].getVelocity(), 0.02);
            }
            // use kinematics to figure out heading
            simHeading = Rotation2d.fromRadians(kinematics.toTwist2d(getWheelPositions()).dtheta);
            poseEstimator.update(simHeading, getWheelPositions());
        } else {
            // --- update real robot ---
            poseEstimator.update(gyro.getRotation2d(), getWheelPositions());
        }

        // telemetry
        field.setRobotPose(poseEstimator.getEstimatedPosition());
        posePublisher.set(poseEstimator.getEstimatedPosition());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Yaw", () -> gyro.getYaw(), null);
    }
}
