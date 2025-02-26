package frc8768.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import frc8768.robot.util.LogUtil;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class Elevator {
    private static final double GEAR_RATIO = 16;
    private static final double TRAVEL_PER_ROT = (1.432*Math.PI)/GEAR_RATIO;
    private static final SparkBaseConfig ELEVATOR_BASE_CONFIG = new SparkFlexConfig()
            .idleMode(SparkBaseConfig.IdleMode.kBrake);
    private final SparkFlex elevatorMotor2, elevatorMotor1;
    private final RelativeEncoder elevatorEncoder;
    private ElevatorState currState = ElevatorState.ZERO;
    private boolean zeroed = false;

    public Elevator() {
        elevatorMotor1 = new SparkFlex(17, SparkLowLevel.MotorType.kBrushless);
        elevatorMotor2 = new SparkFlex(18, SparkLowLevel.MotorType.kBrushless);
        elevatorEncoder = elevatorMotor1.getEncoder();

        elevatorMotor1.configure(new SparkFlexConfig().apply(ELEVATOR_BASE_CONFIG).inverted(true), SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        elevatorMotor2.configure(new SparkFlexConfig().apply(ELEVATOR_BASE_CONFIG).follow(elevatorMotor1, true),
                SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        LogUtil.registerDashLogger(this::dashLog);
    }

    public void zeroElevator() {
        currState = ElevatorState.ZERO;

        zeroed = false;
        while(!(elevatorMotor1.getOutputCurrent() > 15)) {
            elevatorMotor1.set(-0.05);
        }

        elevatorMotor1.set(0);
        elevatorEncoder.setPosition(0);
        zeroed = true;
    }

    private double getPosition() {
        return (elevatorEncoder.getPosition() * TRAVEL_PER_ROT)*2;
    }

    /**
     * Up is positive, down is negative
     */
    public void moveToState(ElevatorState state) {
        currState = state;
    }

    public void tick() {
        if(zeroed) {
            if (!MathUtil.isNear(currState.targetPosition, getPosition(), 1/16d)) {
                if (currState.targetPosition > getPosition()) {
                    elevatorMotor1.set(0.2);
                } else {
                    elevatorMotor1.set(-0.05);
                }
            } else {
                stop();
            }
        } else {
            zeroElevator();
        }
    }

    public void stop() {
        elevatorMotor1.set(0.02);
    }

    public ElevatorState state() {
        return currState;
    }

    public enum ElevatorState {
        ZERO(0),
        L1(0),
        L2(11),
        L3(27),
        L4(46);

        final double targetPosition;

        ElevatorState(double heightInches) {
            this.targetPosition = heightInches;
        }
    }

    private Map<String, Object> dashLog() {
        HashMap<String, Object> map = new HashMap<>();
        map.put("Elevator Position", getPosition());
        map.put("Elevator State", currState.name());
        return map;
    }
}
