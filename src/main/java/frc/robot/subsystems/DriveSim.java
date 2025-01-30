package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.*;

import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;

import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

/** Handles actions for simulating the drivetrain. */
public class DriveSim {
    private Rotation2d simHeading = new Rotation2d();
    private SparkMax[] motors;
    // sim motors. gear reduction could technically be applied here, but it was easier to do manually.
    // a con though is that the simgui velocities are way off, but this is counteracted by manually
    // putting accurate ones to smartdashboard.
    private DCMotor neo = DCMotor.getNEO(1);
    private SparkSim[] sparkSims;
    // used for velocity calculations
    private DCMotorSim[] motorSims =
        new DCMotorSim[] {
            new DCMotorSim(LinearSystemId.createDCMotorSystem(neo, 6.66E-15, 1 / kGearRatio), neo),
            new DCMotorSim(LinearSystemId.createDCMotorSystem(neo, 6.66E-15, 1 / kGearRatio), neo),
            new DCMotorSim(LinearSystemId.createDCMotorSystem(neo, 6.66E-15, 1 / kGearRatio), neo),
            new DCMotorSim(LinearSystemId.createDCMotorSystem(neo, 6.66E-15, 1 / kGearRatio), neo),
        };
    private SparkRelativeEncoderSim[] encodersSims;
    
    /** Requires all motors from the mecanum drivetrain. */
    public DriveSim(SparkMax flMotor, SparkMax frMotor, SparkMax rlMotor, SparkMax rrMotor) {
        motors = new SparkMax[] {flMotor, frMotor, rlMotor, rrMotor};
        sparkSims =
            new SparkSim[] {
                new SparkSim(flMotor, neo),
                new SparkSim(frMotor, neo),
                new SparkSim(rlMotor, neo),
                new SparkSim(rrMotor, neo)
            };
        encodersSims =
            new SparkRelativeEncoderSim[] {
                new SparkRelativeEncoderSim(flMotor),
                new SparkRelativeEncoderSim(frMotor),
                new SparkRelativeEncoderSim(rlMotor),
                new SparkRelativeEncoderSim(rrMotor)
            };
    }

    /** Update the drive simulation. */
    public void updateSimulation(
        MecanumDriveWheelPositions wheelPositions,
        MecanumDrivePoseEstimator poseEstimator
    ) {
        for (int i = 0; i < motorSims.length; ++i) {
            // calculate sim measurements (mainly velocity) from input voltage
            motorSims[i].setInputVoltage(motors[i].getAppliedOutput() * 12);
            motorSims[i].update(0.02);
            // apply it to spark
            sparkSims[i].iterate(motorSims[i].getAngularVelocityRPM() / kGearRatio, RoboRioSim.getVInVoltage(), 0.02);
            encodersSims[i].iterate(motorSims[i].getAngularVelocityRPM() / kGearRatio, 0.02);
        }
        // use kinematics to figure out heading
        simHeading = Rotation2d.fromRadians(kDriveKinematics.toTwist2d(wheelPositions).dtheta);
        poseEstimator.update(simHeading, wheelPositions);
    }

    /** Returns the heading of the robot in simulation. */
    public Rotation2d getHeading() {
        return simHeading;
    }
}
