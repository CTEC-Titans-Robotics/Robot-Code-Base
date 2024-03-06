package frc8768.robot.operators;

import frc8768.robot.Robot;

/**
 * Abstract class designed for Operators.
 * Anything universal to operators goes in here.
 * <p>
 * IMPORTANT: OPERATORS ARE THREADED! For every operator a thread is made.
 * It will process one operator without stalling other operators.
 */
public abstract class Operator {
    /**
     * The thread the operator runs on
     */
    private Thread opThread;

    /**
     * Operator constructor.
     *
     * @param name The name of the operator.
     */
    public Operator(String name) {
        opThread = new Thread(this::runLoop);
        opThread.setName(String.format("%s Thread", name));
    }

    /**
     * Start the thread, do not call this method more than once per-operator.
     */
    public void init() {
        opThread.start();
    }

    /**
     * While loop wrapper
     */
    public void runLoop() {
        while(true) {
            if(!Robot.getInstance().isTeleopEnabled()) {
                continue;
            }
            run();
        }
    }

    /**
     * The operator-specific code
     */
    public abstract void run();

    /**
     * Should an essential subsystem's thread die, restart it.
     */
    public void reviveThread() {
        String origName = opThread.getName().replace(" Thread", "");
        opThread = new Thread(this::runLoop);
        opThread.setName(String.format("%s Thread", origName));

        opThread.start();
    }

    /**
     * Check thread status
     * @return output of {@link java.lang.Thread} isAlive() function for the op thread
     */
    public boolean isAlive() {
        return opThread.isAlive();
    }
}
