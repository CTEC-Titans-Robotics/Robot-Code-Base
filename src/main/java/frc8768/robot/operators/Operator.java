package frc8768.robot.operators;

import edu.wpi.first.wpilibj.GenericHID;
import frc8768.robot.Robot;

/*
   Abstract class designed for Operators.
   Anything universal to operators goes in here.

   OPERATORS ARE THREADED! For every operator a thread is made.
   It will process one operator without stalling other operators.
 */
public abstract class Operator {
    // The thread the operator runs on
    private final Thread opThread;
    // A controller, can be a XboxController or anything deriving from GenericHID
    protected final GenericHID hid;

    /*
        Name: name of the operator
        Controller: The controller object
     */
    public Operator(String name, GenericHID controller) {
        opThread = new Thread(this::runLoop);
        opThread.setName(String.format("%s Thread", name));

        hid = controller;
    }

    public boolean isRobotTeleop() {
        return Robot.getInstance().isTeleop();
    }

    /*
        Starts the thread
     */
    public void init() {
        opThread.start();
    }

    public void runLoop() {
        while(true) {
            if (!isRobotTeleop()) {
                continue;
            }
            run();
        }
    }

    public abstract void run();

    public GenericHID getHid() {
        return hid;
    }
}
