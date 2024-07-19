package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Reads user input continuously from a side variety of human interface devices
 * (including, eventually, keyboard input).  The directional inputs are exposed
 * through the three channel querying methods {@link #getForwardBackChannel()},
 * {@link #getLeftRightChannel()}, and {@link #getTurnChannel()}, while any
 * remaining buttons or sliders are exposed through their own methods.
 *
 */
public class InputSubsystem extends SubsystemBase {

    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
    private XboxController xboxController;
    private Joystick joystick;

    /**
     * How often should we look for new human input devices?
     */
    private static final double JOYSTICK_POLLING_INTERVAL_SECONDS = 3.0;

    private double lastCheckTimeSeconds;

    public InputSubsystem() {
        xboxController = null;
        joystick = null;
        lastCheckTimeSeconds = Timer.getFPGATimestamp() - JOYSTICK_POLLING_INTERVAL_SECONDS; // Force initial check
    }

    /**
     * Checks to see if any controllers have been plugged in, and if so, where.
     * To avoid overwhelming the system with exceptions, callers of this
     * function should only do so occasionally.
     */
    private void discoverControllers() {
        final int PORTS_TO_SCAN = 4;

        for (int i = 0; i < PORTS_TO_SCAN; ++i) {
            try {
                if (xboxController == null) {
                    xboxController = new XboxController(i);
                    // Success!
                    System.out.printf("discoverControllers(): Found XBox Controller on port %d\n", i);
                    continue;
                } else {
                    // Is the XBox controller still reachable?
                    if (!xboxController.isConnected()) {
                        System.out.printf("discoverControllers(): XBox Controller was disconnected!\n", i);
                        xboxController = null;
                    }
                }
            } catch (Exception e) {
                // Couldn't initialize an XBox Controller at this port.
                // Not even worth logging.
            }

            try {
                if (joystick != null) {
                    joystick = new Joystick(i);
                    // Success!
                    System.out.printf("discoverControllers(): Found Joystick on port %d\n", i);
                    continue;
                } else {
                    // Is the joystick still reachable?
                    if (!joystick.isConnected()) {
                        System.out.printf("discoverControllers(): Joystick was disconnected!\n", i);
                        joystick = null;
                    }
                }
            } catch (Exception e) { }

            // TODO: Add NetworkTables support.
        }
    }


    @Override
    public String getName() {
        return "Input Subsystem";
    }

    @Override
    public void periodic() {
        super.periodic();

        // Check to see if anything new was plugged in (or removed.)
        double currentTimeSeconds = Timer.getFPGATimestamp();
        if (currentTimeSeconds - lastCheckTimeSeconds >= JOYSTICK_POLLING_INTERVAL_SECONDS) {
            discoverControllers();
            lastCheckTimeSeconds = currentTimeSeconds;
        }
    }

    /**
     * How much does the user want to strafe sideways?  -1.0 is full speed
     * leftward, +1.0 is full speed rightward, and 0.0 means that no strafing
     * should take place.
     *
     * <ul>
     *   <li>On the Xbox controller, this is the left joystick's horizontal channel.</li>
     *   <li>On gaming joysticks, this is the horizontal channel.</li>
     *   <li>For keyboard input, this is controlled by the A and D keys.</li>
     * </ul>
     */
    public double getLeftRightChannel() {
        double xboxLeftRight = (xboxController != null && xboxController.isConnected() ? xboxController.getLeftX() : 0);
        double joystickLeftRight = (joystick != null && joystick.isConnected() ? joystick.getX() : 0);

        // If all devices are plugged in, then they'll all contribute so long as
        // a human is touching them.  This allows for pair driving.
        double leftRight = MathUtil.applyDeadband(xboxLeftRight, Constants.OperatorConstants.JOYSTICK_DEAD_ZONE) +
                           MathUtil.applyDeadband(joystickLeftRight, Constants.OperatorConstants.JOYSTICK_DEAD_ZONE);

        return m_xspeedLimiter.calculate(leftRight);
    }

    /**
     * How much does the user want to drive straight ahead?  -1.0 is full speed
     * backward, +1.0 is full speed forward, and 0.0 means that no movement
     * should occur in that axis.
     *
     * <ul>
     *   <li>On the Xbox controller, this is the left joystick's vertical channel.</li>
     *   <li>On gaming joysticks, this is the vertical channel.</li>
     *   <li>For keyboard input, this is controlled by the W and S keys.</li>
     * </ul>

     */
    public double getForwardBackChannel() {
        double xboxForwardBack = (xboxController != null && xboxController.isConnected() ? xboxController.getLeftY() : 0);
        double joystickForwardBack = (joystick != null && joystick.isConnected() ? joystick.getY() : 0);

        double forwardBack = MathUtil.applyDeadband(xboxForwardBack, Constants.OperatorConstants.JOYSTICK_DEAD_ZONE) +
                             MathUtil.applyDeadband(joystickForwardBack, Constants.OperatorConstants.JOYSTICK_DEAD_ZONE);

        return m_yspeedLimiter.calculate(forwardBack);
    }

    /**
     * How much does the user want to turn?  -1.0 is full speed
     * counterclockwise, +1.0 is full speed clockwise, and 0.0 means not to
     * turn.
     *
     * <ul>
     *   <li>On the Xbox controller, this is the right joystick's horizontal channel.</li>
     *   <li>On gaming joysticks, this is the so-called "Z" channel (produced by twisting the joystick.)</li>
     *   <li>For keyboard input, this is controlled by the LEFT and RIGHT keys.</li>
     * </ul>

     */
    public double getTurnChannel() {
        double xboxTurn = (xboxController != null && xboxController.isConnected() ? xboxController.getRightX() : 0);
        double joystickTurn = (joystick != null && joystick.isConnected() ? joystick.getZ() : 0);

        double turn = MathUtil.applyDeadband(xboxTurn, Constants.OperatorConstants.JOYSTICK_DEAD_ZONE) +
                      MathUtil.applyDeadband(joystickTurn, Constants.OperatorConstants.JOYSTICK_DEAD_ZONE);

        return m_rotLimiter.calculate(turn);
    }
}
