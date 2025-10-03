package frc8768.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc8768.robot.util.Constants;
import frc8768.robot.util.LogUtil;

import java.util.HashMap;
import java.util.Map;
import java.util.logging.Level;

public class ArmSubsystem implements Subsystem {
    private static double angleCANOffset = -31;

    private Thread launchThread = new Thread(this::runLaunch);

    private final SparkMax followerMotor;
    private final SparkMax leaderMotor;
    public SparkClosedLoopController mainPIDMotor;
    private final CANcoder topEncoder;

    private double armAngle;
    private final double minPos = 0;
    private final double maxPos = 103; // 103
    private double peakCurrentTime = 0;
    private long lastPollElapsedTime = 0;

    private ArmStates armState = ArmStates.NOT_LIMITED;

    private final SparkBaseConfig LIFT_MOTOR_BASE = new SparkMaxConfig().inverted(false).idleMode(SparkBaseConfig.IdleMode.kBrake);
    private final SparkBaseConfig FOLLOWER;

    public ArmSubsystem(int followerMotorId, int leaderMotorId, int topEncoderId) {
        followerMotor = new SparkMax(followerMotorId, SparkLowLevel.MotorType.kBrushless);
        leaderMotor = new SparkMax(leaderMotorId, SparkLowLevel.MotorType.kBrushless);
        mainPIDMotor = leaderMotor.getClosedLoopController();

        topEncoder = new CANcoder(topEncoderId);

        FOLLOWER = new SparkMaxConfig().apply(LIFT_MOTOR_BASE)
                .follow(leaderMotor);

        followerMotor.configure(FOLLOWER, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        LogUtil.registerDashLogger(this::getDebugInfo);
        this.launchThread.start();
    }

    public Map<String, Object> getDebugInfo() {
        HashMap<String, Object> debugInfo = new HashMap<>();
        debugInfo.put("Arm Angle", String.valueOf(armAngle));
        debugInfo.put("Leader Current", String.valueOf(leaderMotor.getOutputCurrent()));
        return debugInfo;
    }

    public void stopGearbox() {
        leaderMotor.set(-0.01);
    }

    public void tick() {
        armAngle = topEncoder.getAbsolutePosition().getValue().in(Units.Degree) - angleCANOffset;

        if(leaderMotor.getOutputCurrent() > 60 && peakCurrentTime < 200) {
            this.peakCurrentTime++;
        } else if(peakCurrentTime >= 200) {
            this.armState = ArmStates.CURRENT_STOP;
            this.leaderMotor.set(0);
        } else if(armState != ArmStates.CURRENT_STOP) {
            this.peakCurrentTime = 0;
        }
    }

    public void moveArm(double speed) {
        if(armState == ArmStates.CURRENT_STOP || (armAngle < minPos && speed > 0)) {
            leaderMotor.set(0);
            return;
        }

        leaderMotor.set(speed);
    }

    public boolean launch() {
        if(armState == ArmStates.LAUNCHED || armState == ArmStates.LAUNCHING || armState == ArmStates.CURRENT_STOP) {
            return false;
        }

        armState = ArmStates.LAUNCHING;

        return true;
    }

    private void runLaunch() {
        while(true) {
            try {
                long waitTime = 2 - lastPollElapsedTime;
                Thread.sleep(lastPollElapsedTime == 0 || waitTime < 0 ? 2 : waitTime);
            } catch (InterruptedException e) {
                LogUtil.LOGGER.log(Level.SEVERE, "An operator got interrupted during sleep!");
                break;
            }
            if (armState == ArmStates.LAUNCHING) {
                if (armAngle > maxPos) {
                    armState = ArmStates.LAUNCHED;
                    stopGearbox();
                } else {
                    moveArm(-1);
                }
            }

            if (armState == ArmStates.RESET) {
                if (armAngle < minPos) {
                    armState = ArmStates.NOT_LIMITED;
                    stopGearbox();
                } else {
                    moveArm(0.5);
                }
            }
        }
    }

    public void reset() {
        if(armState != ArmStates.LAUNCHED)
            return;
        armState = ArmStates.RESET;
    }

    public enum ArmStates {
        NOT_LIMITED,
        LAUNCHED,
        LAUNCHING,
        RESET,
        CURRENT_STOP
    }
}