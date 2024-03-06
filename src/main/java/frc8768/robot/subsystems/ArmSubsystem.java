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
    public double overrideAngle = -1;

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

                if(this.currState.isAngleWithinCoarseTolerance(position) && this.overrideAngle == -1) {
                    if(this.currState.isAngleWithinFineTolerance(position)) {
                        //if both within fine and coarse tolerance stop the motor (you have reached your destination!!)
                        this.armMotor.setVoltage(this.currState.holdVoltage);
                        continue;
                    }
                    //if within coarse tolerance but not within fine tolerance move at slower speed until it reaches fine
                    if(this.currState.getDesiredPosition() > position) {
                        this.armMotor.set(0.06); //og value 0.13
                    } else if(this.currState.getDesiredPosition() < position) {
                        this.armMotor.set(-0.06); //og value 0.13
                    }
                }

                //Goes at normal speed if not in Coarse Tolerance
                if(this.currState.getDesiredPosition() > position && this.overrideAngle == -1) {
                    this.armMotor.set(this.currState.speed);
                } else if(this.currState.getDesiredPosition() < position && this.overrideAngle == -1) {
                    this.armMotor.set(-this.currState.speed);
                }

                if(this.overrideAngle != -1) {
                    if(this.overrideAngle > position) {
                        this.armMotor.set(0.13);
                    } else if(this.overrideAngle < position) {
                        this.armMotor.set(-0.13);
                    }
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
        IDLE(84, 2, 5, 0.22, 0),
        LOW(12,2, 4,0.13, 0.24),
        INTAKE(2, 2, 5,0.13, 0.24),
        AMP(95, 2, 5,0.20, 0),
        SPEAKER(30, 2, 5,0.18, 0.24);

        private final double fineTolerance;
        private final double position;
        private  final double coarseTolerance;
        private final double speed;
        private  final  double holdVoltage;

        ArmState(double angle, double fineTolerance, double coarseTolerance, double speed, double holdVoltage) {
            this.position = angle;
            this.fineTolerance = fineTolerance;
            this.coarseTolerance = coarseTolerance;
            this.speed = speed;
            this.holdVoltage = holdVoltage;
        }

        public double getDesiredPosition() {
            return this.position;
        }

        public boolean isAngleWithinCoarseTolerance(double currAngle) {
            return MathUtil.isNear(this.position, currAngle, this.coarseTolerance);
        }
        public boolean isAngleWithinFineTolerance(double currAngle) {
            return MathUtil.isNear(this.position, currAngle, this.fineTolerance);
        }
    }
}
