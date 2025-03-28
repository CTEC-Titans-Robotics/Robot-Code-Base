package frc8768.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc8768.RobotState;
import java.util.function.DoubleSupplier;

public class ApproachReef extends Command {
  private final boolean bSide;
  private Command approachReef;
  private DoubleSupplier levelOffsetSupplier;

  public enum LevelOffsets {
    // metres
    L4_OFFSET(0.14),
    L3_OFFSET(0.1),
    L2_OFFSET(0),
    L1_OFFSET(0);
    public double levelOffset;

    private LevelOffsets(double levelOffset) {
      this.levelOffset = levelOffset;
    }

    public double getLevelOffset() {
      return levelOffset;
    }
  }

  public ApproachReef(DoubleSupplier levelOffsetSupplier, boolean bSide) {
    this.levelOffsetSupplier = levelOffsetSupplier;
    this.bSide = bSide;
  }

  @Override
  public void initialize() {
    approachReef =
        RobotState.getInstance().approachReefCommand(levelOffsetSupplier.getAsDouble(), bSide);
    approachReef.initialize();

    System.out.println(
        "Initializing ApproachReefCommand, offset of "
            + levelOffsetSupplier.getAsDouble()
            + ", bSide is "
            + bSide);
  }

  @Override
  public void execute() {
    try {
      approachReef.execute();
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  @Override
  public boolean isFinished() {
    return approachReef.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    approachReef.end(interrupted);
    approachReef = null;
  }
}
