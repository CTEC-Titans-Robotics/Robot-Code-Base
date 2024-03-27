package frc8768.robot.operators;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc8768.robot.subsystems.SwerveSubsystem;
import frc8768.robot.util.Constants;
import frc8768.robot.util.LogUtil;

public class DrivebaseOperator extends Operator {
    private static final XboxController controller = new XboxController(Constants.driverControllerId);
    private final SwerveSubsystem swerve;
    private Command currCommand;

    public DrivebaseOperator(SwerveSubsystem swerve) {
        super("Drivebase");

        this.swerve = swerve;
        // sparkTank = Robot.getInstance().getSpark();
        // falconTank = Robot.getInstance().getFalcon();

        // Init logging
        LogUtil.registerLogger(swerve::log);
        LogUtil.registerDashLogger(swerve::dashboard);

        this.swerve.getSwerveDrive().stopOdometryThread();
    }


    //RUN TIME
    @Override
    public void run() {
        this.swerve.getSwerveDrive().updateOdometry();
        boolean isRobotRelative = false;
        if(controller.getPOV() != -1){
            isRobotRelative = true;
        }
        // intakeSubsystem.tick();

        // if(controller.getLeftBumper()) {
        //     armSubsystem.up();
        // } else if(controller.getLeftTriggerAxis() > 0.1) {
        //     armSubsystem.down();
        // } else {
        //     armSubsystem.stop();
        // }

        // if(controller.getRightBumperPressed()) {
        //     intakeSubsystem.run();
        // }

        if(controller.getBButtonPressed()) {
            this.swerve.getSwerveDrive().zeroGyro();
        }


        if(controller.getXButtonPressed()) {
            this.swerve.getSwerveDrive().lockPose();
        }

        // Apply controller deadband
        Translation2d translation2d = new Translation2d(
                MathUtil.applyDeadband(-controller.getLeftY() /* For Tank, use controller.getLeftY() */, Constants.controllerDeadband),
                MathUtil.applyDeadband(-controller.getLeftX() /* For Tank, use controller.getRightY() */, Constants.controllerDeadband));

        if((MathUtil.applyDeadband(controller.getLeftX(), Constants.controllerDeadband) != 0 || MathUtil.applyDeadband(controller.getLeftY(), Constants.controllerDeadband) != 0 ||
                MathUtil.applyDeadband(controller.getRightX(), Constants.controllerDeadband) != 0) && currCommand != null) {
            this.currCommand.cancel();
            this.currCommand = null;
        }

        if(controller.getAButtonPressed()) {
            relocate(Constants.FieldWaypoints.AMP.getPose2d());
        } else {
            if(this.currCommand != null && !this.currCommand.isFinished()) {
                return;
            }
            //Robot Relative control
            double speed = 0.25;
            switch(controller.getPOV()){
                case 0 -> translation2d = new Translation2d(2*speed, 0);
                //case 45 -> translation2d = new Translation2d(speed, speed);
                case 90 -> translation2d = new Translation2d(0,speed);
                //case 135 -> translation2d = new Translation2d(-speed, speed);
                case 180 -> translation2d = new Translation2d(2*-speed, 0);
                //case 225 -> translation2d = new Translation2d(-speed, -speed);
                case 270 -> translation2d = new Translation2d(0, -speed);
                //case 315 -> translation2d = new Translation2d(speed, -speed);
            }

            // Swerve Example
            this.swerve.drive(translation2d, MathUtil.applyDeadband(-controller.getRightX(), Constants.controllerDeadband), !isRobotRelative, false, Constants.BOT_CENTER);
        }

        // Tank Example (Falcons)
        // falconTank.drive(translation2d);

        // Tank Example (Spark)
        // sparkTank.drive(translation2d);
    }

    private void relocate(Pose2d desiredLoc) {
        if(this.currCommand != null && !this.currCommand.isFinished()) {
            return;
        }

        this.currCommand = AutoBuilder.pathfindToPose(desiredLoc, new PathConstraints(
                Units.feetToMeters(Constants.SwerveConfig.MAX_SPEED * Constants.SwerveConfig.MAX_SPEED/10),
                Units.feetToMeters(Constants.SwerveConfig.MAX_SPEED * Constants.SwerveConfig.MAX_SPEED),
                Units.degreesToRadians(450D * 450D), Units.degreesToRadians(720D * 720D/10D)));
        this.currCommand.schedule();
    }
}
