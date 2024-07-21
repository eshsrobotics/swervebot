package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class InputSubsystem extends SubsystemBase {

    @Override
    public String getName() {
        return "Input Subsystem";
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
    }

    /**
     * Gets the user's desired forward-backward movement relative to the robot and returns a value from -1.0 to 1.0.
     * @return -1.0 for full backward, 1.0 for full forward, 0 for no movement, or any value in-between.
     */
    public double getForwardBack() {
        return 0;
    }

    public double getLeftRight() {
        return 0;
    }

    public double getTurn() {
        return 0;
    }

    private XboxController xboxController;
    private Joystick mainJoystick;
    private Joystick secondaryJoystick;

    /**
     * A function called every few seconds meant to detect when xbox controllers
     * or joysticks are connected or disconnected, and updates dynamically.
     */
    private void controllerCheck() {
        final int MAX_PORTS_TO_CHECK = 4;
        for (int i = 0; i < MAX_PORTS_TO_CHECK; i++) {
            try {
                if (xboxController == null) {
                    xboxController = new XboxController(i);
                    System.out.printf("controllerCheck: Found xbox controller on port %d.\n", i);
                    continue;
                } else if (!xboxController.isConnected()) {
                    xboxController = null;
                    System.out.printf("controllerCheck: Xbox controller is disconnected.\n");
                    continue;
                }
            } catch (Exception e) {
                // Xbox controller is not connected on port i, not a big deal.
            } 

            // We want to be able to support both 1 and 2 joystick scenarios.
            try {
                Joystick j = new Joystick(i);
                System.out.printf("controllerCheck: Found joystick on port %d.\n", i);

                if (mainJoystick == null && secondaryJoystick == null) {
                    // The first joystick to be connected is assigned as the
                    // main joystick because the main joystick is needed for
                    // both methods of driving.
                    mainJoystick = j;
                    System.out.printf("controllerCheck: Assigning new joystick on port %d to main.\n", i);
                } else if (mainJoystick != null && secondaryJoystick == null) {
                    if (!mainJoystick.isConnected()) {
                        // If a joystick gets disconnected and another joystick
                        // is added, the newest joystick becomes the main.
                        mainJoystick = j;
                        System.out.printf("controllerCheck: Main joystick is disconnected, assigning new joystick on port %d to main.\n", i);
                    } else {
                        // If the main joystick is null and another joystick is
                        // connected, it is assigned as the secondary joystick.
                        secondaryJoystick = j;
                        System.out.printf("controllerCheck: Main joystick is connected and secondary joystick is missing, assigning new joystick on port %d to secondary.\n", i);
                    }
                } else if (mainJoystick == null && secondaryJoystick != null) {
                    if (!secondaryJoystick.isConnected()) {
                        // The disconnected secondary joystick will be handled
                        // by another case.
                        mainJoystick = j;
                        System.out.printf("controllerCheck: Main joystick is missing and secondary joystick is disconnected, assigning new joystick on port %d to main.\n", i);
                    } else {
                        // If there are two joysticks, the most recently added
                        // joystick will always become the secondary. So if the main
                        // joystick is disconnected while the secondary is still
                        // connected and another joystick is added, the secondary
                        // becomes the main and the newest becomes the secondary.
                        mainJoystick = secondaryJoystick;
                        secondaryJoystick = j;
                        System.out.printf("controllerCheck: Main joystick is missing and secondary joystick is connected, assigning secondary to the main joystick and assigning new joystick on port %d to the secondary.\n", i);
                    } 
                } else if (mainJoystick != null && secondaryJoystick != null) {
                    if (!mainJoystick.isConnected() && !secondaryJoystick.isConnected()) {
                        mainJoystick = j;
                        secondaryJoystick = null;
                        System.out.printf("controllerCheck: Two joysticks were disconnected, assigning new joystick on port %d to main.\n", i);
                    } else if (!mainJoystick.isConnected() && secondaryJoystick.isConnected()) {
                        mainJoystick = secondaryJoystick;
                        secondaryJoystick = j;
                        System.out.printf("controllerCheck: Main joystick was disconnected, assigning secondary to the main joystick and assigning new joystick on port %d to the secondary.\n", i);
                    } else if (mainJoystick.isConnected() && !secondaryJoystick.isConnected()) {
                        secondaryJoystick = j;
                        System.out.printf("controllerCheck: Secondary joystick was disconnected, assigning new joystick on port %d to secondary.\n", i);
                    } else {
                        // Already have two working joysticks, so 3rd joystick does nothing.
                    }
                }

            } catch (Exception e) {
                // No joystick at port i, not a big deal.
            }


        }
    }
    
    // TODO We need to call the controller check every few seconds during
    // periodic, and we need to combine the xbox controller, the main joystick,
    // and the secondary joystick into the 3 degrees of freedom: forward-back,
    // left-right, and turn.
    
}
