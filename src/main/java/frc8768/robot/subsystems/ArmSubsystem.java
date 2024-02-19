package frc8768.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.HashMap;
import java.util.Map;

public class ArmSubsystem implements Subsystem {
    private static final double ANGLE_OFFSET = 0;

    private final CANSparkFlex armMotor = new CANSparkFlex(15, CANSparkLowLevel.MotorType.kBrushless);
    private final DutyCycleEncoder armEncoder = new DutyCycleEncoder(0);
    private final Thread positionThread;
    public ArmState currState = ArmState.IDLE;

    public ArmSubsystem() {
        // Configure Motor
        this.armMotor.restoreFactoryDefaults();
        this.armMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        this.armMotor.setInverted(true);
        this.armMotor.burnFlash();

        // Configure Encoder
        this.armEncoder.setDistancePerRotation(41D/360D);

        // Setup Auto-Pose Thread
        this.positionThread = new Thread(() -> {
            while(true) {
                double position = this.getPosition();
                if(this.currState.isAngleWithinTolerance(position)) {
                    this.armMotor.set(0);
                    continue;
                }

                if(this.currState.getDesiredPosition() > position) {
                    this.armMotor.set(0.25);
                } else if(this.currState.getDesiredPosition() < position) {
                    this.armMotor.set(-0.25);
                }
            }
        });
        this.positionThread.setName("Position Thread");
        this.positionThread.start();
    }

    private double getPosition() {
        return (this.armEncoder.getAbsolutePosition() * this.armEncoder.getDistancePerRotation()) - ANGLE_OFFSET;
    }

    public void stop() {
        if(positionThread != null && positionThread.isAlive()) {
            this.positionThread.interrupt();
        }
    }

    public void setDesiredState(ArmState state) {
        this.currState = state;
    }

    /**
     * Dashboard logging
     *
     * @return Map of Name to Value
     */
    public Map<String, String> dashboard() {
        HashMap<String, String> map = new HashMap<>();
        map.put("Current ArmState", currState.name());
        map.put("Arm Position", String.valueOf(this.getPosition()));
        return map;
    }

    public enum ArmState {
        IDLE(45, 1),
        INTAKE(0, 1),
        AMP(85, 1),
        SPEAKER(35, 1);

        private final double tolerance;
        private final double position;

        ArmState(double angle, double tolerance) {
            this.position = angle;
            this.tolerance = tolerance;
        }

        public double getDesiredPosition() {
            return this.position;
        }

        public boolean isAngleWithinTolerance(double currAngle) {
            return MathUtil.isNear(this.position, currAngle, this.tolerance);
        }
    }
}
