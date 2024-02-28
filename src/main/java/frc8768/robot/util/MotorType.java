package frc8768.robot.util;

public enum MotorType {
    SPARKMAX,

    /**
     * Krakens fall under this configuration, https://pro.docs.ctr-electronics.com/_/downloads/en/latest/pdf/ Page 3
     * IF USING KRAKENS, SET P TO 50.0 AND D TO 0.32 AND TUNE FROM THERE
     */
    TALONFX,
    SPARKFLEX
}
