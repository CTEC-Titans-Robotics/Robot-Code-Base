package frc8768.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc8768.robot.util.LogUtil;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;
import java.util.concurrent.locks.ReentrantLock;
import java.util.logging.Level;

public class IntakeSubsystem implements Subsystem {
    private final CANSparkFlex intakeMotor = new CANSparkFlex(16, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkFlex holdMotor = new CANSparkFlex(17, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkFlex shootMotor = new CANSparkFlex(18, CANSparkLowLevel.MotorType.kBrushless);
    private IntakeStage currStage = IntakeStage.IDLE;
    private final AtomicReference<Thread> intakeLock;

    public double overrideShootSpeed = -1;

    public IntakeSubsystem() {
        // Lock Setup
        this.intakeLock = new AtomicReference<>();

        // Configure Motor
        this.intakeMotor.restoreFactoryDefaults();
        this.intakeMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
        this.intakeMotor.setInverted(true);
        this.intakeMotor.burnFlash();

        // Configure Motor
        this.holdMotor.restoreFactoryDefaults();
        this.holdMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
        this.holdMotor.setInverted(false);
        this.holdMotor.burnFlash();

        // Configure Motor
        this.shootMotor.restoreFactoryDefaults();
        this.shootMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
        this.shootMotor.setInverted(false);
        this.shootMotor.burnFlash();
    }

    public void stop() {
        this.shootMotor.set(0);
        this.holdMotor.set(0);
        this.intakeMotor.set(0);
    }

    public void beginStage(IntakeStage stage) {
        if(this.intakeLock.get() != Thread.currentThread() && this.intakeLock.get() != null) {
            LogUtil.LOGGER.log(Level.WARNING, "Intake is locked by another thread, disallowing access from a calling thread.");
            return;
        }
        this.intakeLock.set(Thread.currentThread());

        this.setStage(stage);
    }

    public void releaseLock() {
        if(this.intakeLock.get() == null || this.intakeLock.get() != Thread.currentThread()) {
            return;
        }
        this.intakeLock.set(null);
    }

    private void setStage(IntakeStage stage) {
        if(this.currStage == stage) {
            return;
        }

        this.currStage = stage;

        double desiredSpeed = this.currStage.getDesiredMotorSpeed();
        switch (this.currStage) {
            case IDLE -> {
                this.shootMotor.set(0);
                this.holdMotor.set(0);
                this.intakeMotor.set(0);
            }
            case INTAKE -> {
                this.shootMotor.set(-0.03);

                this.holdMotor.set(0.2);
                this.intakeMotor.set(desiredSpeed);
            }
            case OUTTAKE -> {
                this.shootMotor.set(-0.03);

                this.holdMotor.set(-0.2);
                this.intakeMotor.set(desiredSpeed);
            }
            case AMP, SPEAKER -> {
                this.intakeMotor.set(0);
                this.holdMotor.set(-0.1);
                try {
                    Thread.sleep(150);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                this.shootMotor.set(this.overrideShootSpeed != -1 ? this.overrideShootSpeed : desiredSpeed);
                try {
                    Thread.sleep(750);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                this.holdMotor.set(0.25);
                try {
                    Thread.sleep(750);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }
        }
    }

    public void tick() {
        if(this.intakeLock.get() == null) {
            this.setStage(IntakeStage.IDLE);
        }
    }

    /**
     * Dashboard logging
     *
     * @return Map of Name to Value
     */
    public Map<String, String> dashboard() {
        HashMap<String, String> map = new HashMap<>();
        map.put("Current IntakeStage", currStage.name());
        map.put("Hold Amps", String.valueOf(holdMotor.getOutputCurrent()));
        map.put("Shoot Amps", String.valueOf(shootMotor.getOutputCurrent()));
        map.put("Intake Amps", String.valueOf(intakeMotor.getOutputCurrent()));
        return map;
    }

    public enum IntakeStage {
        INTAKE(0.73, -1),
        OUTTAKE(-0.2, -1),
        SPEAKER(0.80, 40),
        AMP(0.2, 35),
        IDLE(0, -1);

        private final double desiredSpeed;
        private final double ampTrip;

        IntakeStage(double speed, double ampLimit) {
            this.desiredSpeed = speed;
            this.ampTrip = ampLimit;
        }

        public double getDesiredMotorSpeed() {
            return this.desiredSpeed;
        }

        public boolean hasTripped(double drawnCurrent) {
            return drawnCurrent < this.ampTrip;
        }
    }
}
