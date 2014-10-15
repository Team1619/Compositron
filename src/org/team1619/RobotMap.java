package org.team1619;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
    public static final int rightStick_port = 0;                // ID for the right joystick port
    public static final int leftStick_port = 1;                 // ID for the left joystick port
    
    public static final int motorID_rightFrontDrive = 1;        // IDs for the drive motors (for now only 4, need 6)
    public static final int motorID_rightRearDrive = 1;
    public static final int motorID_leftFrontDrive = 1;
    public static final int motorID_leftRearDrive = 1;
    
    public static final int motorID_rollers = 15;                 // ID for the roller motor
    // For example to map the left and right motors, you could define the
    // following variables to use with your drivetrain subsystem.
    // public static final int leftMotor = 1;
    // public static final int rightMotor = 2;
    
    // If you are using multiple modules, make sure to define both the port
    // number and the module. For example you with a rangefinder:
    // public static final int rangefinderPort = 1;
    // public static final int rangefinderModule = 1;
}
