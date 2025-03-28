package frc8768.robot.subsystems.canWatchdog;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc8768.robot.subsystems.rgb.RGB;
import frc8768.robot.subsystems.rgb.RGB.RGBMessages;
///import org.littletonrobotics.junction.Logger;

public class CANWatchdog extends SubsystemBase {
  private CANWatchdogIO io;
  private RGB rgb;
  private boolean hasAllDevices;

  /** Creates a new CANWatchdog. */
  public CANWatchdog(CANWatchdogIO io, RGB rgb) {
    this.io = io;
    this.rgb = rgb;
  }

  @Override
  public void periodic() {
    int[] missingDevices = io.missingDevices();
    hasAllDevices = missingDevices.length == 0;
    RGBMessages.MISSING_CAN_DEVICE.setIsExpired(hasAllDevices);
    ///Logger.recordOutput("CANWatchdog/NumberOfMissingDevices", missingDevices.length);
    ///Logger.recordOutput("CANWatchdog/HasAllDevices", hasAllDevices());
    if (!hasAllDevices()) {
      ///Logger.recordOutput("CANWatchdog/FirstDeviceMissing", missingDevices[0]);
    }
  }

  public void matchStarting() {
    io.matchStarting();
    RGBMessages.MISSING_CAN_DEVICE.setIsExpired(true);
  }

  public boolean hasAllDevices() {
    return hasAllDevices;
  }
}
