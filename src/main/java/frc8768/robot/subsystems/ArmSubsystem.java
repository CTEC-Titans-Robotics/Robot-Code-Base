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
import java.util.concurrent.atomic.AtomicReference;
import java.util.logging.Level;

public class ArmSubsystem implements Subsystem {
    private static final double ANGLE_OFFSET1 = 220.74631751865795;  //220.26588303164704
    private static final double ANGLE_OFFSET2 = 360 - ANGLE_OFFSET1;

    private final CANSparkFlex armMotor = new CANSparkFlex(15, CANSparkLowLevel.MotorType.kBrushless);
    private final DutyCycleEncoder armEncoder = new DutyCycleEncoder(0);
    private final Thread positionThread;
    private final AtomicReference<Thread> armLock;
    public ArmState currState = ArmState.IDLE_AMP;
    private ArmZone zone;
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

        // Setup Auto-Pose Thread (Thread moves arm to the states position )
        this.positionThread = new Thread(() -> {
            while(true) {
                double position = this.getPosition();

                if(this.currState.isAngleWithinCoarseTolerance(position) && this.overrideAngle == -1) {
                    if(this.currState.isAngleWithinFineTolerance(position)) {
                        //if both within fine and coarse tolerance stop the motor (you have reached your destination!!)
                        this.armMotor.set(this.currState.holdPercent); //og value 0.13

                        try {
                            Thread.sleep(20);
                        } catch (InterruptedException e) {
                            throw new RuntimeException(e);
                        }

                        continue;
                    }
                    //if within coarse tolerance but not within fine tolerance move at slower speed until it reaches fine
                    if(this.currState.getDesiredPosition() > position) {
                        this.armMotor.set(0.02); //og value 0.13
                    } else if(this.currState.getDesiredPosition() < position) {
                        this.armMotor.set(-0.02); //og value 0.13
                    }
                }

                //Goes at normal speed if not in Coarse Tolerance
                double speed = this.zone == ArmZone.GRAVITY_ZONE ? this.currState.speed : this.currState.speed/1.5;
                if(this.currState.getDesiredPosition() > position && this.overrideAngle == -1) {
                    this.armMotor.set(speed);
                } else if(this.currState.getDesiredPosition() < position && this.overrideAngle == -1 && this.zone == ArmZone.GRAVITY_ZONE) {
                    this.armMotor.set(-speed*0.5);
                } else if(this.currState.getDesiredPosition() < position && this.overrideAngle == -1) {
                    this.armMotor.set(-speed*1.75);
                }

                if(this.overrideAngle != -1) {
                    if(this.overrideAngle > position && !MathUtil.isNear(position, this.overrideAngle, 2)) {
                        this.armMotor.set(0.12);
                    } else if(this.overrideAngle < position && !MathUtil.isNear(position, this.overrideAngle, 2)) {
                        this.armMotor.set(-0.08);
                    } else {
                        this.armMotor.set(0.02);
                    }
                }

                try {
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }
        });
        this.positionThread.setName("Position Thread");
        this.positionThread.start();
    }

    public double getPosition() {
        if((this.armEncoder.getAbsolutePosition() * this.armEncoder.getDistancePerRotation()) >= ANGLE_OFFSET1 - 5) {
            return Math.abs(this.armEncoder.getAbsolutePosition() * this.armEncoder.getDistancePerRotation() - ANGLE_OFFSET1);
        } else if (this.armEncoder.getAbsolutePosition() * this.armEncoder.getDistancePerRotation() <= ANGLE_OFFSET2) {
            return Math.abs(this.armEncoder.getAbsolutePosition() * this.armEncoder.getDistancePerRotation() + ANGLE_OFFSET2);
        }
        return 0;
    }

    public void stop() {
        if(positionThread != null && positionThread.isAlive()) {
            this.positionThread.interrupt();
        }
    }

    public void setDesiredState(ArmState state) {
        if(this.armLock.get() != Thread.currentThread() && this.armLock.get() != null) {
            LogUtil.LOGGER.log(Level.WARNING, "Arm is locked by " + this.armLock.get().getName() + " thread, disallowing access from " + Thread.currentThread().getName() + " thread.");
            return;
        }
        this.armLock.set(Thread.currentThread());

        this.currState = state;
    }

    public void releaseLock() {
        if(this.armLock.get() != Thread.currentThread()) {
            return;
        }
        this.armLock.set(null);
    }

    public void tick() {
        if(this.armLock.get() == null) {
            this.currState = ArmState.IDLE_AMP;
        }
        this.zone = ArmZone.GRAVITY_ZONE.isInZone(this.getPosition()) ? ArmZone.GRAVITY_ZONE : ArmZone.UP;
    }

    /**
     * Dashboard logging
     *
     * @param map Map of Name to Value
     */
    public void dashboard(Map<String, String> map) {
        map.put("Current ArmState", currState.name());
        map.put("Arm Position", String.valueOf(+
                this.armEncoder.getAbsolutePosition() * this.armEncoder.getDistancePerRotation()));
        map.put("Adjusted Arm Position", String.valueOf(this.getPosition()));
    }

    public enum ArmZone {
        GRAVITY_ZONE(-90, 35),
        UP(35, 180);

        private final double minAngle;
        private final double maxAngle;

        ArmZone(double minAngle, double maxAngle) {
            this.minAngle = minAngle;
            this.maxAngle = maxAngle;
        }

        public boolean isInZone(double armPos) {
            return armPos > this.minAngle && armPos < this.maxAngle;
        }
    }

    public enum ArmState {
        LOW(12,2, 4,0.16, 0.025), // 12
        INTAKE(1, 1, 5,0.16, 0.02), //1
        IDLE_AMP(97, 2, 5,0.25, 0.02), //97
        SPEAKER(33, 2, 6,0.16, 0.025); //35

        private final double fineTolerance;
        private final double position;
        private  final double coarseTolerance;
        private final double speed;
        private  final  double holdPercent;

        ArmState(double angle, double fineTolerance, double coarseTolerance, double speed, double holdPercent) {
            this.position = angle;
            this.fineTolerance = fineTolerance;
            this.coarseTolerance = coarseTolerance;
            this.speed = speed;
            this.holdPercent = holdPercent;
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
