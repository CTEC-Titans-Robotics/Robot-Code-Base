package frc8768.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ArmSubsystem implements Subsystem {
    private final CANSparkMax armMotor;
    private final RelativeEncoder armEncoder;
    private double currAngle;
    private Thread armMovementThread = new Thread();

    public ArmSubsystem(int id) {
        armMotor = new CANSparkMax(id, CANSparkLowLevel.MotorType.kBrushless);
        armMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        armMotor.restoreFactoryDefaults();
        armMotor.setSmartCurrentLimit(25);
        armMotor.burnFlash();

        armEncoder = armMotor.getEncoder();
        armEncoder.setPosition(0);

        armMovementThread.start();
    }

    public void tick() {
        currAngle = armEncoder.getPosition();
        SmartDashboard.putNumber("currAngle", currAngle);
    }

    public void up() {
        if(currAngle > -1) {
            armMotor.set(-0.2); // TODO
        } else {
            stop();
        }
    }

    public void stop() {
        armMotor.stopMotor();
    }

    public void down() {
        if(currAngle < 7.5) {
            armMotor.set(0.05); // TODO
        } else {
            stop();
        }
    }

    public void dropArm() {
        while(currAngle < 7.5) {
            armMotor.set(0.05);
        }
        stop();
    }

    public void raiseArm() {
        while(currAngle > -1) {
            armMotor.set(-0.2);
        }
        stop();
    }
}
