package frc8768.robot.subsystems;

import com.ctre.phoenix.sensors.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.HashMap;
import java.util.Map;

public class ArmSubsystem implements Subsystem {
    private static double angleCANOffset = 27.58;

    private CANSparkMax followerMotor;
    private CANSparkMax leaderMotor;
    public SparkMaxPIDController mainPIDMotor;
    private CANCoder topEncoder;
    private DigitalInput limitSwitch;
    private SparkMaxAbsoluteEncoder gearboxEncoder;

    private double armAngle;
    private Thread magicButtonThread;
    private double minPos = -80;
    private double maxPos = 20;
    private boolean isExtendedPastThreshold = false;
    private boolean isExtendedPastThreshold2 = false;
    private ArmStates armState = ArmStates.NOT_LIMITED;
    private ArmExtensionSubsystem armExtension;

    public ArmSubsystem(int followerMotorId, int leaderMotorId, int topEncoderId, int limitSwitchId, ArmExtensionSubsystem armExtension) {
        this.armExtension = armExtension;

        followerMotor = new CANSparkMax(followerMotorId, CANSparkMaxLowLevel.MotorType.kBrushless);
        leaderMotor = new CANSparkMax(leaderMotorId, CANSparkMaxLowLevel.MotorType.kBrushless);
        mainPIDMotor = leaderMotor.getPIDController();

        mainPIDMotor.setP(0.01);
        mainPIDMotor.setI(0);
        mainPIDMotor.setD(0.01);
        mainPIDMotor.setIZone(0);

        mainPIDMotor.setPositionPIDWrappingEnabled(true);
        mainPIDMotor.setPositionPIDWrappingMaxInput(maxPos);
        mainPIDMotor.setPositionPIDWrappingMinInput(minPos);

        gearboxEncoder = leaderMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        gearboxEncoder.setPositionConversionFactor(1);

        topEncoder = new CANCoder(topEncoderId);
        armConfigAngleEncoder();

        limitSwitch = new DigitalInput(limitSwitchId);

        followerMotor.follow(leaderMotor);
    }

    private void armConfigAngleEncoder() {
        CANCoderConfiguration armConfig = new CANCoderConfiguration();
        armConfig.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
        armConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        armConfig.sensorTimeBase = SensorTimeBase.PerSecond;
        armConfig.sensorCoefficient = 0.087890625;

        topEncoder.configFactoryDefault();
        topEncoder.configAllSettings(armConfig);

        topEncoder.setPositionToAbsolute();
        followerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        leaderMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public void magicButton(double position) {
        if(magicButtonThread != null) {
            if(magicButtonThread.isAlive()) {
                return;
            }
        }
        magicButtonThread = new Thread(() -> {
            Timer time = new Timer();
            time.start();
            while (true) {
                double runtime = 2;
                double tolerance = 4;
                double armAngle = topEncoder.getAbsolutePosition() - angleCANOffset;

                if (!limitSwitch.get()) {
                    stopGearbox();
                    break;
                }
                if (armAngle >= 20) {
                    stopGearbox();
                    break;
                }
                if (armAngle > position + tolerance && time.get() < runtime ) {
                    leaderMotor.set(0.55);
                } else if (armAngle < position - tolerance && time.get() < runtime) {
                    leaderMotor.set(-0.55);
                } else {
                    time.stop();
                    stopGearbox();
                    break;
                }
            }
        });
        magicButtonThread.start();
    }

    public void stopGearbox() {
        leaderMotor.set(-0.01);
    }

    public void tick() {
        armAngle = topEncoder.getAbsolutePosition() - angleCANOffset;
        armState = armAngle >= maxPos ? ArmStates.MAXIMUM_REACHED :
                (armAngle <= minPos || !limitSwitch.get()) ? ArmStates.MINIMUM_REACHED : ArmStates.NOT_LIMITED;
        isExtendedPastThreshold = armAngle >= minPos/* Small Value */ && armAngle <= -61/*Large Value*/ && armExtension.getExtDistance() > 0.39;
        isExtendedPastThreshold2 = armAngle >= -61/* Small Value */ && armAngle <= -47/*Large Value*/ && armExtension.getExtDistance() > 1.5;
        armExtension.isExtendedPastThreshold = armAngle >= -84 && armAngle <= -61 && armExtension.getExtDistance() > 0.7;
    }

    public void zeroArm() {
        gearboxEncoder.setZeroOffset(gearboxEncoder.getPosition());
        leaderMotor.set(0.1);
        leaderMotor.set(0);
        mainPIDMotor.setReference(10, CANSparkMax.ControlType.kPosition);
    }

    public void moveArm(double speed) {
        if((armState == ArmStates.MAXIMUM_REACHED && speed < 0) || ((armState == ArmStates.MINIMUM_REACHED || (isExtendedPastThreshold || isExtendedPastThreshold2)) && speed > 0)) {
            leaderMotor.set(0);
            return;
        }
        leaderMotor.set(speed);
    }

    public Map<String, String> getDebugInfo() {
        HashMap<String, String> debugInfo = new HashMap<>();
        debugInfo.put("Arm Angle", String.valueOf(armAngle));
        debugInfo.put("Extension Distance", String.valueOf(armExtension.getExtDistance()));
        return debugInfo;
    }
}
