package frc8768.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc8768.robot.subsystems.Arm;

public class Intake extends Command{

    private final Arm arm;


    public Intake(Arm arm) {
        this.arm = arm;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        this.arm.moveToState(Arm.ArmState.INTAKE);
            if(arm.isAtRotation()) {
            arm.spinIntake(false);
            }
        }
    }
