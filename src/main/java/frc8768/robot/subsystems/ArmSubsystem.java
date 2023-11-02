package frc8768.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ArmSubsystem implements Subsystem {
    private final CANSparkMax armMotor;

    public ArmSubsystem(int id) {
        armMotor = new CANSparkMax(id, CANSparkMaxLowLevel.MotorType.kBrushless);
    }
}
