/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;






public class GripSubsystem extends SubsystemBase {
  SpeedController m_shooterLeft;
  SpeedController m_shooterRight;
  SpeedControllerGroup m_shooter;
  DoubleSolenoid s_grip;
  boolean shooterState;
  boolean solenoidState;
  SpeedController m_gripLift;
  boolean gripState;
  Timer timer;
  boolean elevatorState;
  SpeedController m_leftClimber;
  SpeedController m_rightClimber;
  SpeedController m_elevator;
  boolean lowerElevatorState;
  SpeedController m_lowerElevator;
  DigitalInput s_limitswitchtop;
  DigitalInput s_limitswitchbottom;
  DigitalInput s_limitswitchlowel;
  boolean s_limSwValue;
  boolean s_limSwValue1;
  boolean s_limSwValue2;
  AnalogPotentiometer a_potentiometer;
  double a_potVal;

  public GripSubsystem(){ //creates the motor controllers, solenoids and other objects
    m_shooterLeft = new VictorSP(0); //motor
    m_shooterLeft.setInverted(true); //inverts the value of the motor
    m_shooterRight = new VictorSP(1); //motor
    m_shooter = new SpeedControllerGroup(m_shooterLeft, m_shooterRight); //groups the two motors created above
    s_grip = new DoubleSolenoid(2, 3); //solenoid
    shooterState = false;
    solenoidState = false;
    gripState = false;
    elevatorState = false;
    lowerElevatorState = false;
    m_gripLift = new VictorSP(4);
    m_leftClimber = new VictorSP(5);
    m_rightClimber = new VictorSP(2);
    m_elevator = new WPI_TalonSRX(3);
    m_lowerElevator = new WPI_TalonSRX(5);
    s_limitswitchbottom = new DigitalInput(1); //bottom of grip elevator
    s_limitswitchtop = new DigitalInput(2);//top of grip elevator
    s_limitswitchlowel = new DigitalInput(9);//lower elevator
    a_potentiometer = new AnalogPotentiometer(0, -270, 150); //potentiometer for the grip
    


  }
  public void shooting(boolean isPressed, boolean isReleased){ //shooting and intake with toggle
    if(isPressed){
      if(shooterState)
        m_shooter.set(0.75);
      else
        m_shooter.set(-0.75);

      shooterState = !shooterState;
    }
    else if(isReleased)
      m_shooter.set(0.0);
    
  }

  public void gripSolenoid(boolean isPressed, boolean isReleased){ //open and closing of the grip with toggle
    if(isPressed){
      if(solenoidState)
      s_grip.set(Value.kForward);
      else
      s_grip.set(Value.kReverse);

      solenoidState = !solenoidState;
    }
   
    
  }
  public void gripLift(boolean isPressed, boolean isReleased){ //lifting and lowering the grip with toggle
    a_potVal = a_potentiometer.get();
    System.out.println(a_potVal);

    if(isPressed && a_potVal<111 && a_potVal>0){ 

      if(gripState ){
      // timer.start();
      // while(timer.get()<2.1){
      m_gripLift.set(0.75);
      // }
      // timer.stop();
      // timer.reset();
      }

      else{
        // timer.start();
      // while(timer.get()<2.1){
      m_gripLift.set(-0.75);
      // }
      // timer.stop();
      // timer.reset();
      }

        gripState = !gripState;
    }

    else if(isReleased)
    m_gripLift.set(0.0);
    
  }
  public void elevator(boolean isPressed, boolean isReleased){ //makes the gripper elevator go up and down using toggle
    s_limSwValue = s_limitswitchtop.get();
    s_limSwValue1 = s_limitswitchbottom.get();
    

    if(isPressed && !s_limSwValue){ 
      if(elevatorState)
      m_elevator.set(0.75);
      else
      m_elevator.set(-0.75);

        elevatorState = !elevatorState;
    }
    else if(isReleased)
    m_elevator.set(0.0);
  }
  public void lowerElevator(boolean isPressed, boolean isReleased){ //makes the elevator go up and down using toggle
    if(isPressed){
      if(lowerElevatorState)
      m_lowerElevator.set(0.75);
      else
      m_lowerElevator.set(-0.75);

      lowerElevatorState = !lowerElevatorState;
    }
    else if(isReleased)
    m_lowerElevator.set(0.0);
  }
  
}
  
