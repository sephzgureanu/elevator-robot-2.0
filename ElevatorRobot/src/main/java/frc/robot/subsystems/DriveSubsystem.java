/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.*;



/**
 * Add your docs here.
 */
public class DriveSubsystem extends SubsystemBase {

  DifferentialDrive m_drive;
  // SpeedController m_falcon;
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public DriveSubsystem(){
    SpeedController m_frontLeft = new WPI_TalonSRX(6);
    SpeedController m_rearLeft = new WPI_TalonSRX(1);
    SpeedControllerGroup m_left = new SpeedControllerGroup(m_frontLeft, m_rearLeft);
 
    SpeedController m_frontRight = new WPI_TalonSRX(4);
    SpeedController m_rearRight = new WPI_TalonSRX(7);
    SpeedControllerGroup m_right = new SpeedControllerGroup(m_frontRight, m_rearRight);
    // m_falcon = new WPI_TalonFX(0);
 
    m_drive = new DifferentialDrive(m_left, m_right);
    }
    
    public void tankDrive(double leftSpeed, double rightSpeed){
      m_drive.tankDrive(leftSpeed, rightSpeed);
    }

  
  

}
