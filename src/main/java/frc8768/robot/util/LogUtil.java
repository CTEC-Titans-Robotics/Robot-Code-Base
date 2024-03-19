package frc8768.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * Center for all things logging. Dashboard, or printing, its up to you.
 * Example usage can be seen in Drivebase Operator and SwerveSubsystem
 */
public class LogUtil {
    /**
     * Logging instance, change the parameter to your robots name
     */
    public static final Logger LOGGER = Logger.getLogger("Robot-Code-Base");

    private static final List<Consumer<Map<String, String>>> DASHBOARD_LOGGERS = new ArrayList<>();
    private static final List<Consumer<List<String>>> LOGGERS = new ArrayList<>();

    /**
     * @param logger Adds a logger to be printed to SmartDashboard, should return Map with String types.
     */
    public static void registerDashLogger(Consumer<Map<String, String>> logger) {
        DASHBOARD_LOGGERS.add(logger);
    }

    /**
     * @param logger Adds a logger to be printed to the logging instance.
     */
    public static void registerLogger(Consumer<List<String>> logger) {
        LOGGERS.add(logger);
    }

    /**
     * Runs each log function to print the supplied state
     */
    public static void run() {
        HashMap<String, String> map = new HashMap<>();
        for(Consumer<Map<String, String>> supplier : DASHBOARD_LOGGERS) {
            supplier.accept(map);
            for(Map.Entry<String, String> entry : map.entrySet()) {
                boolean ret = SmartDashboard.putString(entry.getKey(), entry.getValue());
                if (!ret) {
                    LOGGER.log(Level.WARNING, String.format("%s was already added with a different type!", entry.getKey()));
                }
            }
        }

        ArrayList<String> strings = new ArrayList<>();
        for(Consumer<List<String>> supplier : LOGGERS) {
            supplier.accept(strings);
        }
        for(String str : strings) {
            LOGGER.log(Level.INFO, str);
        }
    }
}
