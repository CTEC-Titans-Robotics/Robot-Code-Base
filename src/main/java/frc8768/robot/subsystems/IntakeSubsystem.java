package frc8768.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class IntakeSubsystem implements Subsystem {
    private final CANSparkMax intakeMotor;

    public IntakeSubsystem(int id) {
        intakeMotor = new CANSparkMax(id, CANSparkMaxLowLevel.MotorType.kBrushless);
    }
}
