package frc8768.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc8768.robot.util.LogUtil;

import java.util.HashMap;
import java.util.Map;
import java.util.logging.Level;

public class IntakeSubsystem implements Subsystem {
    private final CANSparkFlex intakeMotor = new CANSparkFlex(16, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkFlex holdMotor = new CANSparkFlex(17, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkFlex shootMotor = new CANSparkFlex(18, CANSparkLowLevel.MotorType.kBrushless);
    private IntakeStage currStage = IntakeStage.IDLE;

    public IntakeSubsystem() {
        // Configure Motor
        this.intakeMotor.restoreFactoryDefaults();
        this.intakeMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
        this.intakeMotor.setInverted(true);
        this.intakeMotor.burnFlash();

        // Configure Motor
        this.holdMotor.restoreFactoryDefaults();
        this.holdMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
        this.holdMotor.setInverted(true);
        this.holdMotor.burnFlash();

        // Configure Motor
        this.shootMotor.restoreFactoryDefaults();
        this.shootMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
        this.shootMotor.setInverted(true);
        this.shootMotor.burnFlash();
    }

    public void stop() {
        this.shootMotor.set(0);
        this.holdMotor.set(0);
        this.intakeMotor.set(0);
    }

    public void setStage(IntakeStage stage) {
        if(this.currStage == stage) {
            LogUtil.LOGGER.log(Level.WARNING, "Stage %s was already set, discarding...", this.currStage.name());
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
                this.holdMotor.set(0);
                this.shootMotor.set(0);

                this.intakeMotor.set(desiredSpeed);
            }
            case HOLD -> {
                this.holdMotor.set(0);

                this.intakeMotor.set(desiredSpeed);
                this.shootMotor.set(-desiredSpeed);
            }
            case AMP, SPEAKER -> {
                this.intakeMotor.set(0);

                this.holdMotor.set(-0.12);
                this.shootMotor.set(desiredSpeed);
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                this.holdMotor.set(desiredSpeed);
            }
        }
    }

    public boolean isActive() {
        return this.currStage == IntakeStage.SPEAKER || this.currStage == IntakeStage.AMP || this.currStage == IntakeStage.INTAKE;
    }

    public boolean stageTripped() {
        return switch (this.currStage) {
            case IDLE, HOLD -> false;
            case INTAKE -> this.currStage.hasTripped(this.holdMotor.getOutputCurrent());
            case AMP, SPEAKER -> this.currStage.ampTrip < this.shootMotor.getOutputCurrent();
        };
    }

    public void tick() {
        if(stageTripped()) {
            switch (this.currStage) {
                case INTAKE -> setStage(IntakeStage.HOLD);
                case AMP, SPEAKER -> setStage(IntakeStage.IDLE);
            }
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
        SPEAKER(0.7, 20),
        AMP(0.25, 12),
        HOLD(0.1, -1),
        INTAKE(0.4, 26),
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
            return drawnCurrent >= ampTrip;
        }
    }
}
