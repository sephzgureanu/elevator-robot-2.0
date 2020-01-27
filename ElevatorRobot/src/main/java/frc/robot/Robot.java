/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.TurnLeft;
import frc.robot.commands.TurnRight;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripSubsystem;

import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DigitalInput;
// import com.revrobotics.*;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  
  
  DriveSubsystem drive_subsystem = new DriveSubsystem();
  GripSubsystem grip_subsystem = new GripSubsystem();
  //TurnRight turn_right = new TurnRight(auto_subsystem, drive_subsystem);
  // private static final int deviceID = 1;
  // private CANSparkMax m_motorSparkMax;
  Joystick joy1 = new Joystick(0);
  // SpeedController m_falcon;


  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.*
   */
  @Override
  public void robotInit() {
    //m_autonomousCommand = new TurnRight(drive_subsystem);
    m_autonomousCommand = new TurnLeft(drive_subsystem);


    // m_motorSparkMax = new CANSparkMax(deviceID, MotorType.kBrushless);

    // m_falcon = new WPI_TalonFX(0);
   

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    //m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();}

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    
    
    
  }

  @Override
  public void teleopInit() {
    
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

 
  @Override
  public void teleopPeriodic() {

    double joystickLeft = joy1.getRawAxis(1); //analog stick left
    double joystickRight = joy1.getRawAxis(5); //analog stick right
    boolean shootingButton = joy1.getRawButtonPressed(1); //square
    boolean shotReleased = joy1.getRawButtonReleased(1); //square
    boolean gripButton = joy1.getRawButtonPressed(2); //x
    boolean gripButtonReleased = joy1.getRawButtonReleased(2); //x
    boolean gripLiftButton = joy1.getRawButtonPressed(3); //circle
    boolean gripLiftButtonReleased = joy1.getRawButtonReleased(3); //circle
    boolean elevatorButton = joy1.getRawButtonPressed(4); //triangle
    boolean elevatorButtonReleased = joy1.getRawButtonReleased(4); //triangle
    boolean lowElButton = joy1.getRawButtonPressed(5); //l1
    boolean lowElButtonReleased = joy1.getRawButtonReleased(5); //l1
    // double falconStick = joy1.getRawAxis(4);
    // falconStick = falconStick/2 + 0.5;
    
    // boolean intakeButton = joy1.getRawButton(4); //triangle 
    // double neoStick = joy1.getRawAxis(3);
   
    
    drive_subsystem.tankDrive(joystickLeft,joystickRight);
    grip_subsystem.shooting(shootingButton, shotReleased); //square
    grip_subsystem.gripSolenoid(gripButton, gripButtonReleased); //x
    grip_subsystem.gripLift(gripLiftButton,gripLiftButtonReleased); //circle
    grip_subsystem.elevator(elevatorButton,elevatorButtonReleased); //triangle
    grip_subsystem.lowerElevator(lowElButton,lowElButtonReleased); //l1
    drive_subsystem.getVelocityLeft();
    drive_subsystem.getVelocityRight();
    


    // m_falcon.set(falconStick); //test falcon motor
    // m_motorSparkMax.set(neoStick/2+0.5);

          

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
