package frc8768.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc8768.robot.util.LogUtil;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.logging.Level;

public class ArmSubsystem implements Subsystem {
    private static final double ANGLE_OFFSET1 = 60 ;
    private static final double ANGLE_OFFSET2 = 420 ;

    private final CANSparkFlex armMotor = new CANSparkFlex(15, CANSparkLowLevel.MotorType.kBrushless);
    private final DutyCycleEncoder armEncoder = new DutyCycleEncoder(0);
    private final Thread positionThread;
    private final AtomicReference<Thread> armLock;
    public ArmState currState = ArmState.IDLE;

    public ArmSubsystem() {
        // Lock Setup
        this.armLock = new AtomicReference<>();

        // Configure Motor
        this.armMotor.restoreFactoryDefaults();
        this.armMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        this.armMotor.setInverted(false);
        this.armMotor.burnFlash();

        // Configure Encoder
        this.armEncoder.setDistancePerRotation(360D);
        this.armEncoder.setPositionOffset(this.armEncoder.getAbsolutePosition() - Units.degreesToRotations(129.50758223768955));

        // Setup Auto-Pose Thread (Thread moves arm to the states position )
        this.positionThread = new Thread(() -> {
            while(true) {
                double position = this.getPosition();
                if(this.currState.isAngleWithinTolerance(position)) {
                    this.armMotor.set(0);
                    continue;
                }

                if(this.currState.getDesiredPosition() > position) {
                    this.armMotor.set(0.12); //og value 0.13
                } else if(this.currState.getDesiredPosition() < position) {
                    this.armMotor.set(-0.12); //og value 0.13
                }
            }
        });
        this.positionThread.setName("Position Thread");
        this.positionThread.start();
    }

    public double getPosition() {
         if((this.armEncoder.getAbsolutePosition() * this.armEncoder.getDistancePerRotation()) <= 65)
         {
             return ANGLE_OFFSET1 - (this.armEncoder.getAbsolutePosition() * this.armEncoder.getDistancePerRotation());
         } else {
             return ANGLE_OFFSET2 - (this.armEncoder.getAbsolutePosition() * this.armEncoder.getDistancePerRotation());
         }

//        return this.armEncoder.getDistance();
    }

    public void stop() {
        if(positionThread != null && positionThread.isAlive()) {
            this.positionThread.interrupt();
        }
    }

    public void setDesiredState(ArmState state) {
        if(this.armLock.get() != Thread.currentThread() && this.armLock.get() != null) {
            LogUtil.LOGGER.log(Level.WARNING, "Arm is locked by another thread, disallowing access from thread %s",
                    Thread.currentThread().getName());
            return;
        }
        this.armLock.set(Thread.currentThread());
        this.currState = state;
    }

    public void releaseLock() {
        if(this.armLock.get() == null || this.armLock.get() != Thread.currentThread()) {
            return;
        }
        this.armLock.set(null);
    }

    public void tick() {
        if(this.armLock.get() == null) {
            this.currState = ArmState.IDLE;
        }
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
        IDLE(84, 2),
        LOW(15,2),
        INTAKE(2, 2),
        AMP(95, 2),
        SPEAKER(25, 2);

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
