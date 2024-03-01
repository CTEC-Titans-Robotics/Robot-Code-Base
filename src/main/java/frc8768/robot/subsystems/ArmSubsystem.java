package frc8768.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.HashMap;
import java.util.Map;

public class ArmSubsystem implements Subsystem {
    private static final double ANGLE_OFFSET = -60;

    private final CANSparkFlex armMotor = new CANSparkFlex(15, CANSparkLowLevel.MotorType.kBrushless);
    private final DutyCycleEncoder armEncoder = new DutyCycleEncoder(0);
    private final Thread positionThread;
    private final ProfiledPIDController armController;
    private final ArmFeedforward armFeedforward;
    public ArmState currState = ArmState.IDLE;

    public ArmSubsystem() {
        // Configure Motor
        this.armMotor.restoreFactoryDefaults();
        this.armMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        this.armMotor.setInverted(false);
        this.armMotor.burnFlash();

        // Configure Encoder
        this.armEncoder.setDistancePerRotation(360D);

        // Configure feedforward
        this.armFeedforward = new ArmFeedforward(0.0001, 0.53, 0.69, 0.09);

        // Setup PIDController
        this.armController = new ProfiledPIDController(6.8, 0.0, 0.0,
                new TrapezoidProfile.Constraints(Units.degreesToRadians(5), Units.degreesToRadians(10)));
        this.armController.setTolerance(Units.degreesToRadians(2)); // Degrees, once PID is fine tuned this should be fine
        this.armController.enableContinuousInput(Units.degreesToRadians(0), Units.degreesToRadians(360));

        // Setup Auto-Pose Thread
        this.positionThread = new Thread(() -> {
            while(true) {
                double position = Units.degreesToRadians(this.getPosition());
                double newSetpoint = Units.degreesToRadians(this.currState.getDesiredPosition());

                this.armController.setGoal(newSetpoint);

                double voltage = this.armController.calculate(position) +
                        this.armFeedforward.calculate(newSetpoint - Units.degreesToRadians(2),
                                this.armController.getSetpoint().velocity);
                if(this.armController.atSetpoint() || position > Units.degreesToRadians(100) || position < Units.degreesToRadians(0)) {
                    this.armMotor.setVoltage(0);
                } else {
                    this.armMotor.setVoltage(voltage);
                }
            }
        });
        this.positionThread.setName("Position Thread");
        this.positionThread.start();
    }

    private double getPosition() {
        double pos = (-this.armEncoder.getAbsolutePosition() * this.armEncoder.getDistancePerRotation()) - ANGLE_OFFSET;
        if(pos >= -300 && pos < -5) {
            pos += 360;
        }
        return pos;
    }

    public void stop() {
        if(positionThread != null && positionThread.isAlive()) {
            this.positionThread.interrupt();
        }
    }

    public void manualMovement(double speed) {
        // stops pid loop so manual motion possible
        stop();
        this.armMotor.set(speed);
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
        map.put("Arm Setpoint PID", String.valueOf(this.armController.getSetpoint()));
        map.put("Arm Tolerance PID", String.valueOf(this.armController.getPositionTolerance()));
        return map;
    }

    public enum ArmState {
        IDLE(85),
        INTAKE(0),
        AMP(100),
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
