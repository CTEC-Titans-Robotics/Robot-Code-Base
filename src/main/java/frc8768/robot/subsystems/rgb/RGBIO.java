package frc8768.robot.subsystems.rgb;

import frc8768.robot.subsystems.rgb.RGBConstants.RGBMessage;
///import org.littletonrobotics.junction.AutoLog;

public interface RGBIO {
  ///@AutoLog
  class RGBIOInputs {}

  default void updateInputs(RGBIOInputs inputs) {}

  default void displayMessage(RGBMessage lightMessage) {}

  default void clear() {}
}
