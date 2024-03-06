package frc8768.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ClimberSubsystem implements Subsystem {
    private final CANSparkFlex rightMotor;
    private final CANSparkFlex leftMotor;

    public ClimberSubsystem() {
        this.rightMotor = new CANSparkFlex(20, CANSparkLowLevel.MotorType.kBrushless);
        this.leftMotor = new CANSparkFlex(19, CANSparkLowLevel.MotorType.kBrushless);

        // Configure right
        this.rightMotor.restoreFactoryDefaults();
        this.rightMotor.setInverted(false);
        this.rightMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        this.rightMotor.burnFlash();

        // Configure left
        this.leftMotor.restoreFactoryDefaults();
        this.leftMotor.setInverted(false);
        this.leftMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        this.leftMotor.follow(this.rightMotor);
        this.leftMotor.burnFlash();
    }

    public void init() {
        while (!(this.rightMotor.getOutputCurrent() > 22)) {
            this.rightMotor.set(-0.04);
        }
        this.rightMotor.set(-0.01);
    }

    public void up() {
        this.rightMotor.set(0.08);
    }

    public void down() {
        this.rightMotor.set(-0.1);
    }

    public void stop() {
        this.rightMotor.set(-0.01);
    }
}
