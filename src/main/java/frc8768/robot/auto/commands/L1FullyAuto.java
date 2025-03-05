package frc8768.robot.auto.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc8768.robot.subsystems.Arm;
import frc8768.robot.subsystems.SwerveSubsystem;

public class L1FullyAuto extends Command {
    private final SwerveSubsystem swerve;
    private final Arm arm;
    private AutoState state;

    public L1FullyAuto(SwerveSubsystem swerveSubsystem, Arm arm) {
        this.swerve = swerveSubsystem;
        this.arm = arm;
    }

    @Override
    public void initialize() {
        super.initialize();

        for(AutoState state1 : AutoState.values()) {
            state1.resetTimer();
        }

        state = AutoState.INTAKE;
    }

    @Override
    public void execute() {
        super.execute();

        switch (state) {
            case INTAKE -> {
                if(!state.started()) {
                    state.startTimer();
                    arm.spinIntake(false);
                } else if(state.hasElapsed()) {
                    state = AutoState.INTAKE_STOP;
                }
            }
            case INTAKE_STOP -> {
                if(!state.started()) {
                    state.startTimer();
                    arm.stopIntake();
                } else if(state.hasElapsed()) {
                    state = AutoState.MOVE;
                }
            }
            case MOVE -> {
                if(!state.started()) {
                    state.startTimer();
                    swerve.move(-0.1, 0, 0);
                } else if(state.hasElapsed()) {
                    state = AutoState.STOP;
                } else {
                    swerve.move(-0.1, 0, 0);
                }
            }
            case STOP -> {
                if(!state.started()) {
                    state.startTimer();
                    swerve.move(0, 0, 0);
                } else if(state.hasElapsed()) {
                    state = AutoState.ARM_L1;
                }
            }
            case ARM_L1 -> {
                if(!state.started()) {
                    state.startTimer();
                    arm.moveToState(Arm.ArmState.L1);
                } else if(state.hasElapsed()) {
                    state = AutoState.OUTTAKE;
                }
            }
            case OUTTAKE -> {
                if(!state.started()) {
                    state.startTimer();
                    arm.spinIntake(true);
                } else if(state.hasElapsed()) {
                    state = AutoState.STOP_OUTTAKE;
                }
            }
            case STOP_OUTTAKE -> {
                if(!state.started()) {
                    state.startTimer();
                    arm.stopIntake();
                } else if(state.hasElapsed()) {
                    state = AutoState.DONE;
                    arm.stopIntake();
                    arm.moveToState(Arm.ArmState.ZERO);
                }
            }
        }

        /*
        arm.spinIntake(false);

        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        arm.stopIntake();

        Timer timer = new Timer();
        timer.reset();
        timer.start();
        while(!timer.hasElapsed(3.5)) {
            swerve.move(-0.1, 0, 0);
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
        swerve.move(0, 0, 0);
        arm.moveToState(Arm.ArmState.L1);

        while(!arm.isAtRotation()) {
            arm.tick();
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        arm.spinIntake(true);

        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        arm.stopIntake();
        arm.moveToState(Arm.ArmState.ZERO);
         */
    }

    @Override
    public boolean isFinished() {
        return state == AutoState.DONE;
    }

    private enum AutoState {
        INTAKE(0.5),
        INTAKE_STOP(0),
        MOVE(3.5),
        STOP(0),
        ARM_L1(1),
        OUTTAKE(1),
        STOP_OUTTAKE(0),
        DONE(0);

        private final Timer timer = new Timer();
        private double seconds;

        AutoState(double seconds) {
            this.seconds = seconds;
        }

        public void startTimer() {
            timer.reset();
            timer.start();
        }

        public void resetTimer() {
            timer.stop();
            timer.reset();
        }

        public boolean hasElapsed() {
            return timer.hasElapsed(seconds);
        }

        public boolean started() {
            return timer.isRunning();
        }
    }
}
