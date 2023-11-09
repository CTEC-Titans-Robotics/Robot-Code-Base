package frc8768.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxRelativeEncoder;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ArmSubsystem implements Subsystem {
    private final CANSparkMax armMotor;
    private final SparkMaxRelativeEncoder armEncoder;
    private double currAngle;
    private Thread armMovementThread = new Thread();

    public ArmSubsystem(int id) {
        armMotor = new CANSparkMax(id, CANSparkMaxLowLevel.MotorType.kBrushless);
        armMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        armEncoder = (SparkMaxRelativeEncoder) armMotor.getEncoder();
        armEncoder.setPosition(0);

        armMovementThread.start();
    }

    public void tick() {
        currAngle = armEncoder.getPosition();
    }

    public void up() {
        if(currAngle < 0) {
            armMotor.set(0.3); // TODO
        } else {
            stop();
        }
    }

    public void stop() {
        armMotor.set(0);
    }

    public void down() {
        if(currAngle > -90) {
            armMotor.set(-0.3); // TODO
        } else {
            stop();
        }
    }
}
