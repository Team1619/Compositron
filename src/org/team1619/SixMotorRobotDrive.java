/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package org.team1619;

import com.sun.squawk.util.MathUtils;
import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.MotorSafetyHelper;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.can.CANNotInitializedException;
import edu.wpi.first.wpilibj.can.CANTimeoutException;
import edu.wpi.first.wpilibj.communication.UsageReporting;
import edu.wpi.first.wpilibj.parsing.IUtility;

/**
 *
 * @author DanielHathcock
 */
public class SixMotorRobotDrive implements MotorSafety, IUtility{
    protected MotorSafetyHelper m_safetyHelper;

    /**
     * The location of a motor on the robot for the purpose of driving
     */
    public static class MotorType {

        /**
         * The integer value representing this enumeration
         */
        public final int value;
        static final int kForeLeft_val = 0;
        static final int kForeRight_val = 1;
        static final int kMidLeft_val = 2;
        static final int kMidRight_val = 3;
        static final int kRearLeft_val = 4;
        static final int kRearRight_val = 5;
       
        public static final MotorType kForeLeft = new MotorType(kForeLeft_val);
        
        public static final MotorType kForeRight = new MotorType(kForeRight_val);
        
        public static final MotorType kMidLeft = new MotorType(kMidLeft_val);
        
        public static final MotorType kMidRight = new MotorType(kMidRight_val);
        
        public static final MotorType kRearLeft = new MotorType(kRearLeft_val);
       
        public static final MotorType kRearRight = new MotorType(kRearRight_val);

        
        
        private MotorType(int value) {
            this.value = value;
        }
    }
    
    public static final double kDefaultExpirationTime = 0.1;
    public static final double kDefaultSensitivity = 0.5;
    public static final double kDefaultMaxOutput = 1.0;
    protected static final int kMaxNumberOfMotors = 6;
    protected final int m_invertedMotors[] = new int[6];
    protected double m_sensitivity;
    protected double m_maxOutput;
    protected SpeedController m_foreLeftMotor;
    protected SpeedController m_foreRightMotor;
    protected SpeedController m_midLeftMotor;
    protected SpeedController m_midRightMotor;
    protected SpeedController m_rearLeftMotor;
    protected SpeedController m_rearRightMotor;
    protected boolean m_allocatedSpeedControllers;
    protected boolean m_isCANInitialized = true;
    protected static boolean kArcadeRatioCurve_Reported = false;
    protected static boolean kArcadeStandard_Reported = false;
   
    /* 
    Shouldn't be needed
    
    protected static boolean kTank_Reported = false;
    protected static boolean kMecanumCartesian_Reported = false;
    protected static boolean kMecanumPolar_Reported = false;
    */
    
    //
    public SixMotorRobotDrive(SpeedController foreLeftMotor, SpeedController midLeftMotor, SpeedController rearLeftMotor, 
            SpeedController foreRightMotor, SpeedController midRightMotor, SpeedController rearRightMotor) 
    {
        if (foreLeftMotor == null || midLeftMotor == null || rearLeftMotor == null || foreRightMotor == null 
                || midRightMotor == null || rearRightMotor == null) 
        {
            m_foreLeftMotor = m_midLeftMotor = m_rearLeftMotor = m_foreRightMotor = m_midRightMotor = m_rearRightMotor = null;
            throw new NullPointerException("Null motor provided");
        }
        m_foreLeftMotor = foreLeftMotor;
        m_midLeftMotor = midLeftMotor;
        m_rearLeftMotor = rearLeftMotor;
        m_foreRightMotor = foreRightMotor;
        m_midRightMotor = midRightMotor;
        m_rearRightMotor = rearRightMotor;
        
        m_sensitivity = kDefaultSensitivity;
        m_maxOutput = kDefaultMaxOutput;
        
        for (int i = 0; i < kMaxNumberOfMotors; i++) {
            m_invertedMotors[i] = 1;
        }
        m_allocatedSpeedControllers = false;
	setupMotorSafety();
	drive(0, 0);
    }
    
    public void drive(double outputMagnitude, double curve) {
        double leftOutput, rightOutput;
        
        if(!kArcadeRatioCurve_Reported){
            UsageReporting.report(UsageReporting.kResourceType_RobotDrive, getNumMotors(), UsageReporting.kRobotDrive_ArcadeRatioCurve);
            kArcadeRatioCurve_Reported = true;
        }
        if (curve < 0) {
            double value = MathUtils.log(-curve);
            double ratio = (value - m_sensitivity) / (value + m_sensitivity);
            if (ratio == 0) {
                ratio = .0000000001;
            }
            leftOutput = outputMagnitude / ratio;
            rightOutput = outputMagnitude;
        } else if (curve > 0) {
            double value = MathUtils.log(curve);
            double ratio = (value - m_sensitivity) / (value + m_sensitivity);
            if (ratio == 0) {
                ratio = .0000000001;
            }
            leftOutput = outputMagnitude;
            rightOutput = outputMagnitude / ratio;
        } else {
            leftOutput = outputMagnitude;
            rightOutput = outputMagnitude;
        }
        setLeftRightMotorOutputs(leftOutput, rightOutput);
    }
    
    /**
     * Arcade drive implements single stick driving.
     * Given a single Joystick, the class assumes the Y axis for the move value and the X axis
     * for the rotate value.
     * (Should add more information here regarding the way that arcade drive works.)
     * @param stick The joystick to use for Arcade single-stick driving. The Y-axis will be selected
     * for forwards/backwards and the X-axis will be selected for rotation rate.
     * @param squaredInputs If true, the sensitivity will be decreased for small values
     */
    public void arcadeDrive(GenericHID stick, boolean squaredInputs) {
        // simply call the full-featured arcadeDrive with the appropriate values
        arcadeDrive(stick.getY(), stick.getX(), squaredInputs);
    }

    /**
     * Arcade drive implements single stick driving.
     * Given a single Joystick, the class assumes the Y axis for the move value and the X axis
     * for the rotate value.
     * (Should add more information here regarding the way that arcade drive works.)
     * @param stick The joystick to use for Arcade single-stick driving. The Y-axis will be selected
     * for forwards/backwards and the X-axis will be selected for rotation rate.
     */
    public void arcadeDrive(GenericHID stick) {
        this.arcadeDrive(stick, true);
    }

    /**
     * Arcade drive implements single stick driving.
     * Given two joystick instances and two axis, compute the values to send to either two
     * or four motors.
     * @param moveStick The Joystick object that represents the forward/backward direction
     * @param moveAxis The axis on the moveStick object to use for forwards/backwards (typically Y_AXIS)
     * @param rotateStick The Joystick object that represents the rotation value
     * @param rotateAxis The axis on the rotation object to use for the rotate right/left (typically X_AXIS)
     * @param squaredInputs Setting this parameter to true decreases the sensitivity at lower speeds
     */
    public void arcadeDrive(GenericHID moveStick, final int moveAxis,
            GenericHID rotateStick, final int rotateAxis,
            boolean squaredInputs) {
        double moveValue = moveStick.getRawAxis(moveAxis);
        double rotateValue = rotateStick.getRawAxis(rotateAxis);

        arcadeDrive(moveValue, rotateValue, squaredInputs);
    }

    /**
     * Arcade drive implements single stick driving.
     * Given two joystick instances and two axis, compute the values to send to either two
     * or four motors.
     * @param moveStick The Joystick object that represents the forward/backward direction
     * @param moveAxis The axis on the moveStick object to use for forwards/backwards (typically Y_AXIS)
     * @param rotateStick The Joystick object that represents the rotation value
     * @param rotateAxis The axis on the rotation object to use for the rotate right/left (typically X_AXIS)
     */
    public void arcadeDrive(GenericHID moveStick, final int moveAxis,
            GenericHID rotateStick, final int rotateAxis) {
        this.arcadeDrive(moveStick, moveAxis, rotateStick, rotateAxis, true);
    }

    /**
     * Arcade drive implements single stick driving.
     * This function lets you directly provide joystick values from any source.
     * @param moveValue The value to use for forwards/backwards
     * @param rotateValue The value to use for the rotate right/left
     * @param squaredInputs If set, decreases the sensitivity at low speeds
     */
    public void arcadeDrive(double moveValue, double rotateValue, boolean squaredInputs) {
        // local variables to hold the computed PWM values for the motors
        if(!kArcadeStandard_Reported){
            UsageReporting.report(UsageReporting.kResourceType_RobotDrive, getNumMotors(), UsageReporting.kRobotDrive_ArcadeStandard);
            kArcadeStandard_Reported = true;
        }

        double leftMotorSpeed;
        double rightMotorSpeed;

        moveValue = limit(moveValue);
        rotateValue = limit(rotateValue);

        if (squaredInputs) {
            // square the inputs (while preserving the sign) to increase fine control while permitting full power
            if (moveValue >= 0.0) {
                moveValue = (moveValue * moveValue);
            } else {
                moveValue = -(moveValue * moveValue);
            }
            if (rotateValue >= 0.0) {
                rotateValue = (rotateValue * rotateValue);
            } else {
                rotateValue = -(rotateValue * rotateValue);
            }
        }

        if (moveValue > 0.0) {
            if (rotateValue > 0.0) {
                leftMotorSpeed = moveValue - rotateValue;
                rightMotorSpeed = Math.max(moveValue, rotateValue);
            } else {
                leftMotorSpeed = Math.max(moveValue, -rotateValue);
                rightMotorSpeed = moveValue + rotateValue;
            }
        } else {
            if (rotateValue > 0.0) {
                leftMotorSpeed = -Math.max(-moveValue, rotateValue);
                rightMotorSpeed = moveValue + rotateValue;
            } else {
                leftMotorSpeed = moveValue - rotateValue;
                rightMotorSpeed = -Math.max(-moveValue, -rotateValue);
            }
        }

        setLeftRightMotorOutputs(leftMotorSpeed, rightMotorSpeed);
    }

    /**
     * Arcade drive implements single stick driving.
     * This function lets you directly provide joystick values from any source.
     * @param moveValue The value to use for fowards/backwards
     * @param rotateValue The value to use for the rotate right/left
     */
    public void arcadeDrive(double moveValue, double rotateValue) {
        this.arcadeDrive(moveValue, rotateValue, true);
    }
    
    /** Set the speed of the right and left motors.
     * This is used once an appropriate drive setup function is called such as
     * twoWheelDrive(). The motors are set to "leftSpeed" and "rightSpeed"
     * and includes flipping the direction of one side for opposing motors.
     * @param leftOutput The speed to send to the left side of the robot.
     * @param rightOutput The speed to send to the right side of the robot.
     */
    public void setLeftRightMotorOutputs(double leftOutput, double rightOutput) {
        if (m_rearLeftMotor == null || m_rearRightMotor == null) {
            throw new NullPointerException("Null motor provided");
        }

        byte syncGroup = (byte)0x80;

        m_foreLeftMotor.set(limit(leftOutput) * m_invertedMotors[MotorType.kForeLeft_val] * m_maxOutput, syncGroup);
        
        m_midLeftMotor.set(limit(leftOutput) * m_invertedMotors[MotorType.kMidLeft_val] * m_maxOutput, syncGroup);

        m_rearLeftMotor.set(limit(leftOutput) * m_invertedMotors[MotorType.kRearLeft_val] * m_maxOutput, syncGroup);

        m_foreRightMotor.set(-limit(rightOutput) * m_invertedMotors[MotorType.kForeRight_val] * m_maxOutput, syncGroup);
        
        m_midRightMotor.set(limit(leftOutput) * m_invertedMotors[MotorType.kMidRight_val] * m_maxOutput, syncGroup);
        
        m_rearRightMotor.set(-limit(rightOutput) * m_invertedMotors[MotorType.kRearRight_val] * m_maxOutput, syncGroup);

        if (m_isCANInitialized) {
            try {
                CANJaguar.updateSyncGroup(syncGroup);
            } catch (CANNotInitializedException e) {
                m_isCANInitialized = false;
            } catch (CANTimeoutException e) {}
        }

        if (m_safetyHelper != null) m_safetyHelper.feed();
    }
    
    /**
     * Limit motor values to the -1.0 to +1.0 range.
     */
    protected static double limit(double num) {
        if (num > 1.0) {
            return 1.0;
        }
        if (num < -1.0) {
            return -1.0;
        }
        return num;
    }

    /**
     * Invert a motor direction.
     * This is used when a motor should run in the opposite direction as the drive
     * code would normally run it. Motors that are direct drive would be inverted, the
     * drive code assumes that the motors are geared with one reversal.
     * @param motor The motor index to invert.
     * @param isInverted True if the motor should be inverted when operated.
     */
    public void setInvertedMotor(MotorType motor, boolean isInverted) {
        m_invertedMotors[motor.value] = isInverted ? -1 : 1;
    }
    
    /**
     * Set the turning sensitivity.
     *
     * This only impacts the drive() entry-point.
     * @param sensitivity Effectively sets the turning sensitivity (or turn radius for a given value)
     */
    public void setSensitivity(double sensitivity)
    {
            m_sensitivity = sensitivity;
    }

    /**
     * Configure the scaling factor for using RobotDrive with motor controllers in a mode other than PercentVbus.
     * @param maxOutput Multiplied with the output percentage computed by the drive functions.
     */
    public void setMaxOutput(double maxOutput)
    {
            m_maxOutput = maxOutput;
    }
    
    /**
     * Free the speed controllers if they were allocated locally
     */
    public void free() {
        if (m_allocatedSpeedControllers) {
            if (m_foreLeftMotor != null) {
                ((PWM) m_foreLeftMotor).free();
            }
            if (m_foreRightMotor != null) {
                ((PWM) m_foreRightMotor).free();
            }
            if (m_midLeftMotor != null) {
                ((PWM) m_midLeftMotor).free();
            }
            if (m_midRightMotor != null) {
                ((PWM) m_midRightMotor).free();
            }
            if (m_rearLeftMotor != null) {
                ((PWM) m_rearLeftMotor).free();
            }
            if (m_rearRightMotor != null) {
                ((PWM) m_rearRightMotor).free();
            }
        }
    }

    public void setExpiration(double timeout) {
        m_safetyHelper.setExpiration(timeout);
    }

    public double getExpiration() {
        return m_safetyHelper.getExpiration();
    }

    public boolean isAlive() {
        return m_safetyHelper.isAlive();
    }

    public boolean isSafetyEnabled() {
        return m_safetyHelper.isSafetyEnabled();
    }

    public void setSafetyEnabled(boolean enabled) {
        m_safetyHelper.setSafetyEnabled(enabled);
    }
    
    public String getDescription() {
        return "Robot Drive";
    }
    
    public void stopMotor() {
        if (m_foreLeftMotor != null) {
            m_foreLeftMotor.set(0.0);
        }
        if (m_foreRightMotor != null) {
            m_foreRightMotor.set(0.0);
        }
        if (m_midLeftMotor != null) {
            m_midLeftMotor.set(0.0);
        }
        if (m_midRightMotor != null) {
            m_midRightMotor.set(0.0);
        }
        if (m_rearLeftMotor != null) {
            m_rearLeftMotor.set(0.0);
        }
        if (m_rearRightMotor != null) {
            m_rearRightMotor.set(0.0);
        }
    }

    private void setupMotorSafety() {
        m_safetyHelper = new MotorSafetyHelper(this);
        m_safetyHelper.setExpiration(kDefaultExpirationTime);
        m_safetyHelper.setSafetyEnabled(true);
    }

    protected int getNumMotors()
    {
        int motors = 0;
        if (m_foreLeftMotor != null) motors++;
        if (m_foreRightMotor != null) motors++;
        if (m_midLeftMotor != null) motors++;
        if (m_midRightMotor != null) motors++;
        if (m_rearLeftMotor != null) motors++;
        if (m_rearRightMotor != null) motors++;
        return motors;
    }
}
