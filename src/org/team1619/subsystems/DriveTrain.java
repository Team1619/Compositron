/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package org.team1619.subsystems;

import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.can.CANTimeoutException;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.team1619.RobotMap;
import org.team1619.SixMotorRobotDrive;
import org.team1619.commands.Drive;

/**
 *
 * @author jess
 */
public class DriveTrain extends Subsystem {

    private final CANJaguar leftForeDriveMotor;
    private final CANJaguar leftMidDriveMotor;
    private final CANJaguar leftRearDriveMotor;
    private final CANJaguar rightForeDriveMotor;
    private final CANJaguar rightMidDriveMotor;
    private final CANJaguar rightRearDriveMotor;
   
    private final SixMotorRobotDrive driveTrain;
    
    public void initDefaultCommand() {   
        this.setDefaultCommand(new Drive());
    }
    
    public DriveTrain() throws CANTimeoutException {
        leftForeDriveMotor = new CANJaguar(RobotMap.motorID_leftForeDrive);
        leftMidDriveMotor = new CANJaguar(RobotMap.motorID_leftMidDrive);
        leftRearDriveMotor = new CANJaguar(RobotMap.motorID_leftRearDrive);
        rightForeDriveMotor = new CANJaguar(RobotMap.motorID_rightForeDrive);
        rightMidDriveMotor = new CANJaguar(RobotMap.motorID_rightMidDrive);
        rightRearDriveMotor = new CANJaguar(RobotMap.motorID_rightRearDrive);
        
        driveTrain = new SixMotorRobotDrive(leftForeDriveMotor, leftMidDriveMotor, leftRearDriveMotor, 
                rightForeDriveMotor, rightMidDriveMotor, rightRearDriveMotor);
    }
    
    public void drive(Joystick stick)
    {
        driveTrain.arcadeDrive(stick);
    }
    
    public void stop()
    {
        driveTrain.stopMotor();
    }
}
