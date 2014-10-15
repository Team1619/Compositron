/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package org.team1619.subsystems;

import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.can.CANTimeoutException;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.team1619.RobotMap;

/**
 *
 * @author jess
 */
public class DriveTrain extends Subsystem {

    private final CANJaguar rightFrontDriveMotor;
    private final CANJaguar rightRearDriveMotor;
    private final CANJaguar leftFrontDriveMotor;
    private final CANJaguar leftRearDriveMotor;
    private final RobotDrive driveTrain;
    
    protected DriveTrain() throws CANTimeoutException {
        rightFrontDriveMotor = new CANJaguar(RobotMap.motorID_rightFrontDrive);
        rightRearDriveMotor = new CANJaguar(RobotMap.motorID_rightRearDrive);
        leftFrontDriveMotor = new CANJaguar(RobotMap.motorID_leftFrontDrive);
        leftRearDriveMotor = new CANJaguar(RobotMap.motorID_leftRearDrive);
        
        driveTrain = new RobotDrive(leftFrontDriveMotor, leftRearDriveMotor, rightFrontDriveMotor, rightRearDriveMotor);
    }
    
    protected void initDefaultCommand() {   
        
    }

}
