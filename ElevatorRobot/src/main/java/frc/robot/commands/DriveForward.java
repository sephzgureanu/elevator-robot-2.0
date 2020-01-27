/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveForward extends CommandBase {
  double encoderValInit;
  double encoderVal;
  double distance;
  private final DriveSubsystem drive_subsystem;

  public DriveForward(DriveSubsystem subsystem) {
    drive_subsystem = subsystem;
    addRequirements(drive_subsystem);
    
  }

  
  @Override
  public void initialize() {
    //get value from encoder
    encoderValInit = drive_subsystem.getVelocityLeft();
  }

  
  @Override
  public void execute() {
    //get value encoder *18.85 = distance 
    //set both motors to 1.0  v=d/t
    encoderVal = drive_subsystem.getVelocityLeft();
    distance = encoderVal*18.85;
    drive_subsystem.tankDrive(1.0, 1.0);

  }

  @Override
  public void end(boolean interrupted) {
    //stop motors
    drive_subsystem.tankDrive(0.0, 0.0);
  }

  
  @Override
  public boolean isFinished() {
    return false;
    //if distance is hit
  }

}
