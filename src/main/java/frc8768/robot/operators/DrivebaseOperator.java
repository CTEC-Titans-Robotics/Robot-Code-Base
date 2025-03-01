package frc8768.robot.operators;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc8768.robot.subsystems.Arm;
import frc8768.robot.subsystems.Elevator;
//import frc8768.robot.subsystems.GroundIndefector;
import frc8768.robot.subsystems.SwerveSubsystem;
import frc8768.robot.util.Constants;
import frc8768.robot.util.LogUtil;

/**
 * Operator for driving the bot
 */
public class DrivebaseOperator extends Operator {
    private final XboxController controller;
    private final SwerveSubsystem swerve;
   // private final GroundIndefector indefector;
    private final Elevator elevator;
    private final Arm arm;

    // private final TankSubsystemSpark sparkTank;
    // private final TankSubsystemFalcon falconTank;

    /**
     * Make an instance of the operator
     *
     * @param swerve The required subsystem for this operator.
     */


    //public DrivebaseOperator(XboxController controller, SwerveSubsystem swerve, GroundIndefector indefector, Elevator elevator) {
        public DrivebaseOperator(XboxController controller, SwerveSubsystem swerve, Elevator elevator, Arm arm) {
        super("Drivebase");

        this.swerve = swerve;
        this.controller = controller;

        // sparkTank = Robot.getInstance().getSpark();
        // falconTank = Robot.getInstance().getFalcon();

       // this.indefector = indefector;
        this.elevator = elevator;
            this.arm = arm;

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

        if (controller.getBButtonPressed()) {
            swerve.getSwerveDrive().zeroGyro();
        }

        if (controller.getYButton()) {
            elevator.moveToState(Elevator.ElevatorState.HANG);
            arm.moveToState(Arm.ArmState.L4);
        }

        if (controller.getAButtonPressed()) {
            elevator.moveToState(Elevator.ElevatorState.ZERO);
            arm.moveToState(Arm.ArmState.L4);
        }

        if (controller.getXButton() && controller.getAButton()) {
            elevator.moveDown();
        }

        if (controller.getXButton() && controller.getYButton()) {
            elevator.moveDown();
        }


      /* if(controller.getRightBumperButton() && controller.getRightTriggerAxis() > 0.1) {
            indefector.spinIntake(true);
        } else if (controller.getLeftBumperButton()) {
            indefector.spinIntake(false);
        } else {
            indefector.stopIntake();
        }

        if(controller.getRightBumperButton() && controller.getRightTriggerAxis() > 0.1) {
            // Don't do anything.
        } else if(controller.getRightBumperButton()) {
            indefector.backwards();
        } else if(controller.getRightTriggerAxis() > 0.1) {
            indefector.forward();
        } else {
            indefector.stop();
        }
*/
        double xRobotRelative = 0;
        double yRobotRelative = 0;

        if(controller.getPOV() == 0) {
           if (elevator.state() == Elevator.ElevatorState.L4){
               xRobotRelative = -.05; }
           else
               xRobotRelative = .05;
        } else if (controller.getPOV() == 180) {
            if (elevator.state() == Elevator.ElevatorState.L4){
                xRobotRelative = .05; }
            else
                xRobotRelative = -.05;
        } else if (controller.getPOV() == 90) {
            if (elevator.state() == Elevator.ElevatorState.L4){
                yRobotRelative = -.05; }
            else
                yRobotRelative = .05;
        } else if (controller.getPOV() == 270) {
            if (elevator.state() == Elevator.ElevatorState.L4){
                yRobotRelative = .05; }
            else
                yRobotRelative = -.05;
        }


        double rot = MathUtil.applyDeadband(-controller.getRightX(), Constants.CONTROLLER_DEADBAND);
        if((elevator.state() != Elevator.ElevatorState.ZERO && elevator.state() != Elevator.ElevatorState.L1) || !elevator.isAtTarget()) {
            translation2d = translation2d.times(0.05);
            rot *= 0.05;
        }

        Translation2d robotRelative = new Translation2d(xRobotRelative, yRobotRelative);

        // Swerve Example
        this.swerve.drive(robotRelative.getNorm() == 0 ? translation2d : robotRelative,
                rot,
                robotRelative.getNorm() == 0, true, Constants.BOT_CENTER);




        // Tank Example (Falcons)
        // falconTank.drive(translation2d);

        // Tank Example (Spark)
        // sparkTank.drive(translation2d);
    }
}
