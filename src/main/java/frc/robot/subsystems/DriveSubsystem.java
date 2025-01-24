package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
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
import static frc.robot.Constants.DrivetrainConstants.*;
import static frc.robot.Constants.MotorControllers.*;

public class DriveSubsystem extends SubsystemBase {
    private SparkMax flMotor = new SparkMax(kFrontLeft, MotorType.kBrushless);
    private SparkMax frMotor = new SparkMax(kFrontRight, MotorType.kBrushless);
    private SparkMax rlMotor = new SparkMax(kRearLeft, MotorType.kBrushless);
    private SparkMax rrMotor = new SparkMax(kRearRight, MotorType.kBrushless);
    private MecanumDrive drivetrain = new MecanumDrive(flMotor, rlMotor, frMotor, rrMotor);
    private SparkMax[] motors = new SparkMax[] {flMotor, frMotor, rlMotor, rrMotor};
    private MecanumDriveKinematics kinematics = kDriveKinematics;
    private MecanumDrivePoseEstimator poseEstimator;
    private AHRS gyro;
    private final double kWheelDiameterMeters = kWheelDiameter.baseUnitMagnitude();

    // closed loop control
    private PIDController flController = new PIDController(kP, 0, kD);
    private PIDController frController = new PIDController(kP, 0, kD);
    private PIDController rlController = new PIDController(kP, 0, kD);
    private PIDController rrController = new PIDController(kP, 0, kD);
    // placeholder from other robot code. need to find for our robot
    private SimpleMotorFeedforward motorFeedforward = new SimpleMotorFeedforward(0.17472, 2.7572, 0.45109);

    // telemetry & sim
    private Field2d field = new Field2d();
    private StructPublisher<Pose2d> posePublisher =
        NetworkTableInstance.getDefault().getStructTopic("Robot Pose", Pose2d.struct).publish();
    private Rotation2d simHeading = new Rotation2d();
    // sim motors. gear reduction could technically be applied here, but it was easier to do manually.
    // a con though is that the simgui velocities are way off, but this is counteracted by manually
    // putting accurate ones to smartdashboard.
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
            new DCMotorSim(LinearSystemId.createDCMotorSystem(neo, 6.66E-15, 1 / kGearRatio), neo),
            new DCMotorSim(LinearSystemId.createDCMotorSystem(neo, 6.66E-15, 1 / kGearRatio), neo),
            new DCMotorSim(LinearSystemId.createDCMotorSystem(neo, 6.66E-15, 1 / kGearRatio), neo),
            new DCMotorSim(LinearSystemId.createDCMotorSystem(neo, 6.66E-15, 1 / kGearRatio), neo),
        };
    private SparkRelativeEncoderSim[] encodersSims =
        new SparkRelativeEncoderSim[] {
            new SparkRelativeEncoderSim(flMotor),
            new SparkRelativeEncoderSim(frMotor),
            new SparkRelativeEncoderSim(rlMotor),
            new SparkRelativeEncoderSim(rrMotor)
        };

    public DriveSubsystem() {
        // avoid mutating default config by using apply
        SparkBaseConfig config = new SparkMaxConfig().apply(kDefaultNeoConfig);
        // account for gearing. Encoder still reports native units (rotations, rpm)
        // Does not work, at least in simulation, so this is applied manually where needed.
        // the conversion shows up in the gui, but doesn't actually apply? Apparently the firmware
        // is supposed to apply it, so that could be why.
        // config.encoder
        //     .positionConversionFactor(1 / kGearRatio)
        //     .velocityConversionFactor(1 / kGearRatio);
        // config.closedLoop.p(kP);

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

        poseEstimator = new MecanumDrivePoseEstimator(kinematics, gyro.getRotation2d(), getWheelPositions(), new Pose2d(2.5, 1.5, new Rotation2d()));

        // configure pathplanning
        AutoBuilder.configure(
            poseEstimator::getEstimatedPosition,
            poseEstimator::resetPose,
            () -> kinematics.toChassisSpeeds(getWheelSpeeds()),
            (speeds, ff) -> driveRobotRelative(speeds),
            kDriveController,
            kRobotConfig,
            () -> {
                // flip path only if alliance is red
                if (DriverStation.getAlliance().isPresent()) {
                    return DriverStation.getAlliance().orElseThrow() == DriverStation.Alliance.Red;
                }
                return true;
            },
            this
        );

        SmartDashboard.putNumber("kP", SmartDashboard.getNumber("kP", kP));
        SmartDashboard.putNumber("kD", SmartDashboard.getNumber("kD", kD));

        // telemetry
        // SmartDashboard.putData("Field", field);
        // PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
        //     field.setRobotPose(pose);
        // });

        // PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
        //     field.getObject("target pose").setPose(pose);
        // });

        // PathPlannerLogging.setLogActivePathCallback((poses) -> {
        //     field.getObject("path").setPoses(poses);
        // });
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

    public void driveRobotRelative(ChassisSpeeds relativeSpeeds) {
        MecanumDriveWheelSpeeds targetWheelSpeeds = kinematics.toWheelSpeeds(relativeSpeeds);
        // targetWheelSpeeds.desaturate(edu.wpi.first.units.Units.FeetPerSecond.of());
        MecanumDriveWheelSpeeds currentWheelSpeeds = getWheelSpeeds();

        SmartDashboard.putNumber("X Target Speeds mps", relativeSpeeds.vxMetersPerSecond);

        double flFF = motorFeedforward.calculate(targetWheelSpeeds.frontLeftMetersPerSecond);
        double frFF = motorFeedforward.calculate(targetWheelSpeeds.frontRightMetersPerSecond);
        double rlFF = motorFeedforward.calculate(targetWheelSpeeds.rearLeftMetersPerSecond);
        double rrFF = motorFeedforward.calculate(targetWheelSpeeds.rearRightMetersPerSecond);

        // flMotor.getClosedLoopController().setReference(targetWheelSpeeds.frontLeftMetersPerSecond / Math.PI * 60 / kWheelDiameterMeters * 10, ControlType.kVelocity);
        // frMotor.getClosedLoopController().setReference(targetWheelSpeeds.frontRightMetersPerSecond / Math.PI * 60 / kWheelDiameterMeters * 10, ControlType.kVelocity);
        // rlMotor.getClosedLoopController().setReference(targetWheelSpeeds.rearLeftMetersPerSecond / Math.PI * 60 / kWheelDiameterMeters * 10, ControlType.kVelocity);
        // rrMotor.getClosedLoopController().setReference(targetWheelSpeeds.rearRightMetersPerSecond / Math.PI * 60 / kWheelDiameterMeters * 10, ControlType.kVelocity);
        double flOutput = flController.calculate(currentWheelSpeeds.frontLeftMetersPerSecond, targetWheelSpeeds.frontLeftMetersPerSecond);
        double frOutput = frController.calculate(currentWheelSpeeds.frontRightMetersPerSecond, targetWheelSpeeds.frontRightMetersPerSecond);
        double rlOutput = rlController.calculate(currentWheelSpeeds.rearLeftMetersPerSecond, targetWheelSpeeds.rearLeftMetersPerSecond);
        double rrOutput = rrController.calculate(currentWheelSpeeds.rearRightMetersPerSecond, targetWheelSpeeds.rearRightMetersPerSecond);

        flMotor.setVoltage(flOutput + flFF);
        frMotor.setVoltage(frOutput + frFF);
        rlMotor.setVoltage(rlOutput + rlFF);
        rrMotor.setVoltage(rrOutput + rrFF);
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

    // --- SubsystemBase --------------------------------------------------------------------------

    @Override 
    public void periodic() {
        if (!Robot.isReal()) {
            // --- update sims ---
            for (int i = 0; i < motorSims.length; ++i) {
                // calculate sim measurements (mainly velocity) from input voltage
                motorSims[i].setInputVoltage(motors[i].getAppliedOutput() * 12);
                motorSims[i].update(0.02);
                // apply it to spark
                sparkSims[i].iterate(motorSims[i].getAngularVelocityRPM() / kGearRatio, RoboRioSim.getVInVoltage(), 0.02);
                encodersSims[i].iterate(motorSims[i].getAngularVelocityRPM() / kGearRatio, 0.02);
            }
            // use kinematics to figure out heading
            simHeading = Rotation2d.fromRadians(kinematics.toTwist2d(getWheelPositions()).dtheta);
            poseEstimator.update(simHeading, getWheelPositions());
        } else {
            // --- update real robot ---
            poseEstimator.update(gyro.getRotation2d(), getWheelPositions());
        }

        flController.setP(SmartDashboard.getNumber("kP", kP));
        frController.setP(SmartDashboard.getNumber("kP", kP));
        rlController.setP(SmartDashboard.getNumber("kP", kP));
        rrController.setP(SmartDashboard.getNumber("kP", kP));

        flController.setD(SmartDashboard.getNumber("kD", kD));
        frController.setD(SmartDashboard.getNumber("kD", kD));
        rlController.setD(SmartDashboard.getNumber("kD", kD));
        rrController.setD(SmartDashboard.getNumber("kD", kD));

        // telemetry
        field.setRobotPose(poseEstimator.getEstimatedPosition());
        posePublisher.set(poseEstimator.getEstimatedPosition());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Yaw", () -> gyro.getYaw(), null);
        builder.addDoubleProperty("FL Setpoint", flController::getSetpoint, null);
        builder.addDoubleProperty("FR Setpoint", frController::getSetpoint, null);
        builder.addDoubleProperty("RL Setpoint", rlController::getSetpoint, null);
        builder.addDoubleProperty("RR Setpoint", rrController::getSetpoint, null);
        builder.addDoubleProperty("FL Measurement", () -> flMotor.getEncoder().getVelocity() * Math.PI / 60 * kWheelDiameterMeters / kGearRatio, null);
        builder.addDoubleProperty("FR Measurement", () -> frMotor.getEncoder().getVelocity() * Math.PI / 60 * kWheelDiameterMeters / kGearRatio, null);
        builder.addDoubleProperty("RL Measurement", () -> rlMotor.getEncoder().getVelocity() * Math.PI / 60 * kWheelDiameterMeters / kGearRatio, null);
        builder.addDoubleProperty("RR Measurement", () -> rrMotor.getEncoder().getVelocity() * Math.PI / 60 * kWheelDiameterMeters / kGearRatio, null);
        builder.addDoubleProperty("X Speed", () -> kinematics.toChassisSpeeds(getWheelSpeeds()).vxMetersPerSecond, null);
        builder.addDoubleProperty("Y Speed", () -> kinematics.toChassisSpeeds(getWheelSpeeds()).vyMetersPerSecond, null);
    }
}
