package org.usfirst.frc.team4915.robot;

import org.usfirst.frc.team4915.robot.commands.DriveForwardCommand;

import edu.wpi.first.wpilibj.buttons.JoystickButton;

import edu.wpi.first.wpilibj.Joystick;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    // Ports for joysticks
    public static final int DRIVE_STICK_PORT = 0;
    public static final int AUX_STICK_PORT = 1;
    
    public final Joystick m_driveStick = new Joystick(DRIVE_STICK_PORT);
    public final Joystick m_auxStick = new Joystick(AUX_STICK_PORT);

    public OI() {
        JoystickButton m_driveForwardButton = new JoystickButton(m_auxStick, 8);
        m_driveForwardButton.whenPressed(new DriveForwardCommand());
    }
}
