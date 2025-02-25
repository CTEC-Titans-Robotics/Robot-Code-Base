package frc8768.robot.operators;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc8768.robot.subsystems.Elevator;
import frc8768.robot.subsystems.GroundIndefector;
import frc8768.robot.subsystems.SwerveSubsystem;
import frc8768.robot.util.Constants;
import frc8768.robot.util.LogUtil;

/**
 * Operator for driving the bot
 */
public class DrivebaseOperator extends Operator {
    private final XboxController controller;
    private final SwerveSubsystem swerve;
    private final GroundIndefector indefector;
    private final Elevator elevator;
    private Command currCommand;

    // private final TankSubsystemSpark sparkTank;
    // private final TankSubsystemFalcon falconTank;

    /**
     * Make an instance of the operator
     *
     * @param swerve The required subsystem for this operator.
     */
    public DrivebaseOperator(XboxController controller, SwerveSubsystem swerve, GroundIndefector indefector, Elevator elevator) {
        super("Drivebase");

        this.swerve = swerve;
        this.controller = controller;
        // sparkTank = Robot.getInstance().getSpark();
        // falconTank = Robot.getInstance().getFalcon();

        this.indefector = indefector;
        this.elevator = elevator;

        // Init logging
        LogUtil.registerLogger(swerve::log);
        LogUtil.registerDashLogger(swerve::dashboard);
    }

    @Override
    public void run() {
        // Apply controller deadband
        Translation2d translation2d = new Translation2d(
                MathUtil.applyDeadband(-controller.getLeftY() /* For Tank, use controller.getLeftY() */, Constants.CONTROLLER_DEADBAND),
                MathUtil.applyDeadband(-controller.getLeftX() /* For Tank, use controller.getRightY() */, Constants.CONTROLLER_DEADBAND));

        if((MathUtil.applyDeadband(controller.getLeftX(), Constants.CONTROLLER_DEADBAND) != 0 || MathUtil.applyDeadband(controller.getLeftY(), Constants.CONTROLLER_DEADBAND) != 0 ||
                MathUtil.applyDeadband(controller.getRightX(), Constants.CONTROLLER_DEADBAND) != 0) && currCommand != null) {
            this.currCommand.cancel();
            this.currCommand = null;
        }

        if(this.currCommand != null && !this.currCommand.isFinished()) {
            return;
        }

        if (controller.getBButtonPressed()) {
            swerve.getSwerveDrive().zeroGyro();
        }

        if (controller.getAButton()) {
            elevator.down();
        } else if (controller.getYButton()) {
            elevator.up();
        } else {
            elevator.stop();
        }

        if(controller.getRightBumperButton() && controller.getRightTriggerAxis() > 0.1) {
            indefector.spinIntake(true);
        } if (controller.getLeftBumperButton()) {
            indefector.spinIntake(false);
        } else {
            indefector.stopIntake();
        }

        if(controller.getRightBumperButton() && controller.getRightTriggerAxis() > 0.1) {
            // Don't do anything.
        } else if(controller.getRightBumperButton()) {
            indefector.forward();
        } else if(controller.getRightTriggerAxis() > 0.1) {
            indefector.backwards();
        } else {
            indefector.stop();
        }

        double xRobotRelative = 0;
        double yRobotRelative = 0;

        if(controller.getPOV() == 0) {
            xRobotRelative = 0.75;
        } else if (controller.getPOV() == 180) {
            xRobotRelative = -.75;
        }

        if (controller.getPOV() == 90) {
            yRobotRelative = 0.75;
        } else if (controller.getPOV() == 270) {
            yRobotRelative = -.75;
        }

        Translation2d robotRelative = new Translation2d(xRobotRelative, yRobotRelative);

        // Swerve Example
        this.swerve.drive(robotRelative.getNorm() == 0 ? translation2d : robotRelative,
                MathUtil.applyDeadband(-controller.getRightX(), Constants.CONTROLLER_DEADBAND),
                robotRelative.getNorm() == 0, true, Constants.BOT_CENTER);

        // Tank Example (Falcons)
        // falconTank.drive(translation2d);

        // Tank Example (Spark)
        // sparkTank.drive(translation2d);
    }
}
