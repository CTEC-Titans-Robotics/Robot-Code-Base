package frc8768.robot.util;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
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

    private static final List<Supplier<Map<String, Object>>> DASHBOARD_LOGGERS = new ArrayList<>();
    private static final List<Supplier<List<String>>> LOGGERS = new ArrayList<>();

    /**
     * @param logger Adds a logger to be printed to SmartDashboard, should return Map with String types.
     */
    public static void registerDashLogger(Supplier<Map<String, Object>> logger) {
        DASHBOARD_LOGGERS.add(logger);
    }

    /**
     * @param logger Adds a logger to be printed to the logging instance.
     */
    public static void registerLogger(Supplier<List<String>> logger) {
        LOGGERS.add(logger);
    }

    /**
     * Runs each log function to print the supplied state
     */
    public static void run() {
        for(Supplier<Map<String, Object>> supplier : DASHBOARD_LOGGERS) {
            for(Map.Entry<String, Object> entry : supplier.get().entrySet()) {
                if (entry.getValue() instanceof String str) {
                    boolean ret = SmartDashboard.putString(entry.getKey(), str);
                    if (!ret) {
                        LOGGER.log(Level.WARNING, String.format("%s was already added with a different type!", entry.getKey()));
                    }
                } else if(entry.getValue() instanceof Sendable sendable) {
                    SmartDashboard.putData(entry.getKey(), sendable);
                } else if(entry.getValue() instanceof Boolean) {
                    SmartDashboard.putBoolean(entry.getKey(), (Boolean) entry.getValue());
                } else if(entry.getValue() instanceof Number) {
                    SmartDashboard.putNumber(entry.getKey(), ((Number) entry.getValue()).doubleValue());
                } else {
                    LOGGER.log(Level.WARNING, String.format("%s could not be registered as the type was not configured.", entry.getKey()));
                }
            }
        }

        for(Supplier<List<String>> supplier : LOGGERS) {
            for(String log : supplier.get()) {
                LOGGER.log(Level.INFO, log);
            }
        }
    }
}
