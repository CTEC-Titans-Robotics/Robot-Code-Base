package frc8768.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ArmExtensionSubsystem implements Subsystem {
    public boolean locked = false;
    private CANSparkMax extensionMotor;
    private DutyCycleEncoder encoder;
    private boolean hasZeroed = false;
    private ArmStates extState = ArmStates.NOT_LIMITED;
    private double extDistance;

    public ArmExtensionSubsystem(int extensionMotorId, int encoderId) {
        extensionMotor = new CANSparkMax(extensionMotorId, CANSparkMaxLowLevel.MotorType.kBrushless);
        encoder = new DutyCycleEncoder(encoderId);

        extensionMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public double getExtDistance() {
        return this.extDistance;
    }

    public void tick() {
        extDistance = -1 * encoder.getDistance();
        //4.8
        double maxPos = 3.6;
        // 0.1
        double minPos = 0.1;
        extState = extDistance >= maxPos ? ArmStates.MAXIMUM_REACHED :
                extDistance <= minPos ? ArmStates.MINIMUM_REACHED : ArmStates.NOT_LIMITED;
    }

    public void zeroExtension() {
        hasZeroed = true;
        for(int i = 0; i < 9999; i++) {
            extensionMotor.set(-0.1);
            if(extensionMotor.getOutputCurrent() > 30) {
                extensionMotor.stopMotor();
                encoder.reset();
                return;
            }
        }
        encoder.reset();
    }

    public void stopExtension() {
        extensionMotor.set(-0.03);
    }

    public void moveArmExtension(double speed) {
        if(!hasZeroed) {
            zeroExtension();
        }
        if(((extState == ArmStates.MAXIMUM_REACHED || locked) && speed > 0) || (extState == ArmStates.MINIMUM_REACHED && speed < 0)) {
            extensionMotor.set(0);
            return;
        }
        extensionMotor.set(speed);
    }
}
