/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Timer;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;


public class AutoSubsystem extends SubsystemBase {
  DriveSubsystem drive_subsystem = new DriveSubsystem();
  Timer timer;
  PigeonIMU pigeon;
  PigeonIMU.GeneralStatus genStat;
  int loop = 0;
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public AutoSubsystem(){
    timer = new Timer();
    TalonSRX m_frontLeft = new WPI_TalonSRX(6);
    pigeon = new PigeonIMU(m_frontLeft); //the pigeon is connected to the Talon above
    genStat = new PigeonIMU.GeneralStatus();
    }

  public void driveForward(double time){
    
    timer.start();
    if (timer.get() < time){
    drive_subsystem.tankDrive(1.0, 1.0);
    }
    timer.stop();
    timer.reset();
    
    }

  public void driveBackwards(){
    drive_subsystem.tankDrive(-1.0, -1.0);
   }

  public void driveStop(){
    drive_subsystem.tankDrive(0.0, 0.0);
   }

  public void pigeonStat(){
    pigeon.getGeneralStatus(genStat);
    System.out.println("General status " + genStat);
    }

  public double getYaw(){
    if (loop++ > 0){
      loop = 0;
    double[] ypr = new double[3];
    pigeon.getYawPitchRoll(ypr);
    System.out.println("Yaw= " + ypr[0]);}
    }

  public void setAngle(double newAngle){
    pigeon.setYaw(newAngle);
    pigeon.setFusedHeading(newAngle);
   }
  
}
