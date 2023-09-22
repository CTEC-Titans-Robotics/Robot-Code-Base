package frc8768.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ArmExtensionSubsystem implements Subsystem {
    private CANSparkMax extensionMotor;
    private DutyCycleEncoder encoder;

    private double maxPos = 3.6; //4.8
    private double minPos = 0.1; // 0.1
    private double extDistance;
    private ArmStates armExtensionState = ArmStates.NOT_LIMITED;

    public ArmExtensionSubsystem(int extensionMotorId, int encoderId) {
        extensionMotor = new CANSparkMax(extensionMotorId, CANSparkMaxLowLevel.MotorType.kBrushless);
        encoder = new DutyCycleEncoder(encoderId);

        extensionMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public void tick() {
        extDistance = -1 * encoder.getDistance();
        armExtensionState = extDistance >= maxPos ? ArmStates.MAXIMUM_REACHED : extDistance <= minPos ? ArmStates.MINIMUM_REACHED : ArmStates.NOT_LIMITED;
    }

    public void zeroExtension() {
        while(true) {
            extensionMotor.set(-0.05);
            if(extensionMotor.getOutputCurrent() > 20) {
                extensionMotor.stopMotor();
                encoder.reset();
                break;
            }
        }
    }

    public void stopExtension() {
        extensionMotor.set(0);
    }

    public void moveArmExtension(double speed) {
        if((armExtensionState == ArmStates.MAXIMUM_REACHED && speed > 0) || (armExtensionState == ArmStates.MINIMUM_REACHED && speed < 0)) {
            extensionMotor.set(0);
            return;
        }
        extensionMotor.set(speed);
    }
}
