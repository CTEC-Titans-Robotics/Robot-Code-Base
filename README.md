# Robot-Code-Base
Universal codebase for all of our robots, completely free for other teams to use.

# JavaDocs
Documentation can be found [here](https://ctec-titans-robotics.github.io/Robot-Code-Base/).

# Example Implementations
Branches 2023-Athena and 2022 show all our code for those seasons. Feel free to check them out.

# Getting Started
Should you still be confused about what to do after reviewing the other branches, I have outlined how to initialize Swerve
and the Drivebase Operator.

Robot Class:

```
try {
    swerve = new SwerveSubsystem(Constants.SwerveConfig.currentType);
} catch (IOException io) {
    throw new RuntimeException("Swerve failed to create!", io);
}
```

This is the code to create the swerve subsystem. On its own, its not very usefull as it has no comamnds to run on.
What you would *want* to do, is create an Operator as shown in the DrivebaseOperator class.

```
public class DrivebaseOperator extends Operator {
private final SwerveSubsystem swerve;
// private final TankSubsystemSpark sparkTank;
// private final TankSubsystemFalcon falconTank;
private static final XboxController controller = new XboxController(Constants.driverControllerId);

    public DrivebaseOperator() {
        super("Drivebase");

        swerve = Robot.getInstance().getSwerve();
        // sparkTank = Robot.getInstance().getSpark();
        // falconTank = Robot.getInstance().getFalcon();
    }

    @Override
    public void run() {
        swerve.getSwerveDrive().updateOdometry();

        if(controller.getBButtonPressed()) {
            swerve.getSwerveDrive().zeroGyro();
        }

        // Apply controller deadband
        Translation2d translation2d = new Translation2d(
                MathUtil.applyDeadband(-controller.getLeftY() /* For Tank, use controller.getLeftY() */, Constants.controllerDeadband),
                MathUtil.applyDeadband(-controller.getLeftX() /* For Tank, use controller.getRightY() */, Constants.controllerDeadband));

        // Swerve Example
        swerve.drive(translation2d, MathUtil.applyDeadband(-controller.getRightX(), Constants.controllerDeadband), true, false, false);

        // Tank Example (Falcons)
        // falconTank.drive(translation2d);

        // Tank Example (Spark)
        // sparkTank.drive(translation2d);
    }
}
```

Allow me to walk through these lines 1 by 1.

``
super("Drivebase")
``
Every Operator is Threaded. This is just used to name the Thread so we can identify it later.

``
Translation2d translation2d = new Translation2d(
MathUtil.applyDeadband(-controller.getLeftY(), Constants.controllerDeadband),
MathUtil.applyDeadband(-controller.getLeftX(), Constants.controllerDeadband));
``
When using potentiometer analog controllers, there is **always** going to be stick drift. The "Deadzone" is used to
differentiate real from fake input. But if you just do a check for if the deadzone is passed, then you have a chance to go from
0 to 100 very quickly. MathUtil.applyDeadband allows the deadband to not limit it so heavily and makes the input smoother.

A couple of notes, Left Y is for translation, and Left X is for strafing. We invert *every* axis because it will go the opposite way you want it to go.

``
swerve.drive(translation2d, MathUtil.applyDeadband(-controller.getRightX(), Constants.controllerDeadband), true, false, false);
``
This is what drives the swerve subsystem. This specific configuration uses the PIDs supplied in pidfproperties.json,
is field relative, applies the deadband for rotation, and applies the Translation2d.

Of course, Subsystem setups vary depending on what it is designed for. A claw would use Pneumatics, so you create a ClawSubsystem
using Pneumatics, or if you have an arm that goes up and down, then an ArmSubsystem would be wanted.

These can all be added into the corresponding Operator object, so it can be easily accessed.

Finally, to initialize the Operator so it actually *runs*:

Near the end of robotInit in the Robot class:
``<operator object>.init();``

DO NOT INITIALIZE THE SAME OPERATOR TWICE. This will cause an error and crash your program. Operators do not run outside of teleop though, keep that in mind.

# License
We operate under the GNU GPL v3 License. You may redistribute, fork, and use this codebase as your own. You may NOT:

* Make a fork closed source,
* Change the license,
* Claim it is yours and only yours.

Follow these, and CTEC is happy to let you use the codebase.
