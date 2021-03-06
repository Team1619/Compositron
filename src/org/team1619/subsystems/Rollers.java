/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package org.team1619.subsystems;

import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.can.CANTimeoutException;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.team1619.RobotMap;

/**
 *
 * @author jess
 */
public class Rollers extends Subsystem {
    
    private final CANJaguar rollersMotor;
    
    public Rollers() throws CANTimeoutException {
        rollersMotor = new CANJaguar(RobotMap.motorID_rollers);
    }
    
    protected void initDefaultCommand() {
        
    }
    
    protected void setRollers(double speed) throws CANTimeoutException {
        rollersMotor.setX(speed);
    }
    
    protected void stopRollers() throws CANTimeoutException {
        rollersMotor.disableControl();
    }
    
}
