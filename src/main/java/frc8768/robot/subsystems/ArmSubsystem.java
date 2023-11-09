package frc8768.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxRelativeEncoder;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ArmSubsystem implements Subsystem {
    private final CANSparkMax armMotor;
    private final SparkMaxRelativeEncoder armEncoder;
    private double currAngle;

    public ArmSubsystem(int id) {
        armMotor = new CANSparkMax(id, CANSparkMaxLowLevel.MotorType.kBrushless);
        armMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

        armEncoder = (SparkMaxRelativeEncoder) armMotor.getEncoder();
        armEncoder.setPosition(0);
    }

    public void tick() {
        currAngle = armEncoder.getPosition();
    }

    public void up() {
        while(currAngle < 100) {
            armMotor.set(0.3);// TODO
        }
        armMotor.set(0);
    }

    public void down() {
        while(currAngle > 0) {
            armMotor.set(-0.3);// TODO
        }
        armMotor.set(0);
    }
}
