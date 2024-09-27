package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
    private CANSparkMax flMotor = new CANSparkMax(1, MotorType.kBrushless);
    private CANSparkMax frMotor = new CANSparkMax(2, MotorType.kBrushless);
    private CANSparkMax rlMotor = new CANSparkMax(3, MotorType.kBrushless);
    private CANSparkMax rrMotor = new CANSparkMax(4, MotorType.kBrushless);

    public DriveSubsystem() {}

    @Override
    public void periodic() {}
}
