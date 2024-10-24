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
        while (!(this.rightMotor.getOutputCurrent() > 21)) {
            this.rightMotor.set(-0.08);
        }
        this.rightMotor.set(-0.01);
    }

    public void up() {
        if(this.rightMotor.get() != 0.4) {
            this.rightMotor.set(0.4);
        }
    }

    public void down() {
        if(this.rightMotor.get() != -0.4) {
            this.rightMotor.set(-0.4);
        }
    }

    public void stop() {
        if(this.rightMotor.get() != -0.01) {
            this.rightMotor.set(-0.01);
        }
    }
}
