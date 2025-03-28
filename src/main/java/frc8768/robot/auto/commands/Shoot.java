package frc8768.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc8768.robot.subsystems.Arm;
public class Shoot extends Command{

    private final Arm arm;

    public Shoot(Arm arm) {
        this.arm = arm;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {



            arm.spinIntake(true);
        }
    }


