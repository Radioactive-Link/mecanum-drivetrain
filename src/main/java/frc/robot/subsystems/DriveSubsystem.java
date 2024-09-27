package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
    private CANSparkMax flMotor = new CANSparkMax(1, MotorType.kBrushless);
    private CANSparkMax frMotor = new CANSparkMax(2, MotorType.kBrushless);
    private CANSparkMax rlMotor = new CANSparkMax(3, MotorType.kBrushless);
    private CANSparkMax rrMotor = new CANSparkMax(4, MotorType.kBrushless);
    private MecanumDrive drivetrain = new MecanumDrive(flMotor, rlMotor, frMotor, rrMotor);

    public DriveSubsystem() {
        flMotor.restoreFactoryDefaults();
        frMotor.restoreFactoryDefaults();
        rrMotor.restoreFactoryDefaults();
        rlMotor.restoreFactoryDefaults();

        // inverse the right side of the drivetrain
        rrMotor.setInverted(true);
        rlMotor.setInverted(true);
    }

    // --- Public Methods -------------------------------------------------------------------------

    public void driveCartesian(double xSpeed, double ySpeed, double zRotation) {
        drivetrain.driveCartesian(xSpeed, ySpeed, zRotation);
    }

    // --- SubsystemBase --------------------------------------------------------------------------

    @Override
    public void periodic() {}

    @Override
    public void initSendable(SendableBuilder builder) {}
}
