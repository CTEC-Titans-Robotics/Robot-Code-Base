package frc8768.robot.subsystems.canWatchdog;

import java.util.HashSet;
import java.util.Set;

public class CANWatchdogConstants {
  public static final String REQUEST = "http://localhost:1250/?action=getdevices";
  public static final int SCAN_DELAY_MS = 97;

  public class CAN {
    private CAN() {}

    public record CANDevice(int id, String name) {}

    private static final Set<CANDevice> devices = new HashSet<>();
    private static final Set<Integer> ids = new HashSet<>();

    public static int at(int id, String name) {
      devices.add(new CANDevice(id, name));
      ids.add(id);
      return id;
    }

    public static Set<CANDevice> getDevices() {
      return devices;
    }

    public static Set<Integer> getIds() {
      return ids;
    }
  }
}
