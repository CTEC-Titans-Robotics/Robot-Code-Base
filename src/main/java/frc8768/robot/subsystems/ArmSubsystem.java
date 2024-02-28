package frc8768.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.HashMap;
import java.util.Map;

public class ArmSubsystem implements Subsystem {
    private static final double ANGLE_OFFSET = -117.99327894983196;

    private final CANSparkFlex armMotor = new CANSparkFlex(15, CANSparkLowLevel.MotorType.kBrushless);
    private final DutyCycleEncoder armEncoder = new DutyCycleEncoder(0);
    private final Thread positionThread;
    private final PIDController armController;
    public ArmState currState = ArmState.IDLE;

    public ArmSubsystem() {
        // Configure Motor
        this.armMotor.restoreFactoryDefaults();
        this.armMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        this.armMotor.setInverted(false);
        this.armMotor.burnFlash();

        // Configure Encoder
        this.armEncoder.setDistancePerRotation(360D);

        // Setup PIDController
        this.armController = new PIDController(0.001, 0, 0.001);
        this.armController.setTolerance(1); // Degrees, once PID is fine tuned this should be fine

        // Setup Auto-Pose Thread
        this.positionThread = new Thread(() -> {
            while(true) {
                double position = this.getPosition();
                double feed = this.armController.calculate(position, this.currState.getDesiredPosition());
                if(this.armController.atSetpoint()) {
                    this.armMotor.set(0);
                } else {
                    this.armMotor.set(feed);
                }
            }
        });
        this.positionThread.setName("Position Thread");
        this.positionThread.start();
    }

    private double getPosition() {
        return (-this.armEncoder.getAbsolutePosition() * this.armEncoder.getDistancePerRotation()) - ANGLE_OFFSET;
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
        IDLE(84),
        INTAKE(0),
        AMP(85),
        SPEAKER(47.5);

        private final double position;

        ArmState(double angle) {
            this.position = angle;
        }

        public double getDesiredPosition() {
            return this.position;
        }
    }
}
