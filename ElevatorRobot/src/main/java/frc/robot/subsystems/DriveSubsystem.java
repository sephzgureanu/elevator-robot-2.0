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
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;



/**
 * Add your docs here.
 */
public class DriveSubsystem extends SubsystemBase{

  DifferentialDrive m_drive;
  PigeonIMU pigeon;
  PigeonIMU.GeneralStatus genStat;
  int loop = 0;
  double[] ypr;
  // SpeedController m_falcon;
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public DriveSubsystem(){
    SpeedController m_frontLeft = new WPI_TalonSRX(6); //has the pigeon connected in AutoSubsystem
    SpeedController m_rearLeft = new WPI_TalonSRX(1);
    SpeedControllerGroup m_left = new SpeedControllerGroup(m_frontLeft, m_rearLeft);

 
    SpeedController m_frontRight = new WPI_TalonSRX(4);
    SpeedController m_rearRight = new WPI_TalonSRX(7);
    SpeedControllerGroup m_right = new SpeedControllerGroup(m_frontRight, m_rearRight);

    TalonSRX m_frontLeftP = new WPI_TalonSRX(6);
    pigeon = new PigeonIMU(m_frontLeftP); //the pigeon is connected to the Talon above
    genStat = new PigeonIMU.GeneralStatus();
    ypr = new double[3];

   
    // m_falcon = new WPI_TalonFX(0);
 
    m_drive = new DifferentialDrive(m_left, m_right);

    m_drive.tankDrive(0.0, 0.0);
    }
    
    public void tankDrive(double leftSpeed, double rightSpeed){
      m_drive.tankDrive(leftSpeed, rightSpeed);
    }

    public void pigeonStat(){
      pigeon.getGeneralStatus(genStat);
      System.out.println("General status " + genStat);
      }
  
    public double getYaw(){
      //if (loop++ > 0){
        //loop = 0;
        pigeon.getYawPitchRoll(ypr);
        System.out.println("Yaw= " + ypr[0]);//}
        return ypr[0];
        
      }
  
    public void setAngle(double newAngle){
      pigeon.setYaw(newAngle);
      pigeon.setFusedHeading(newAngle);
     }

      

  
  

}
