/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.logging.Logger;

public class ElevatorSubystem extends SubsystemBase {

  /**
   * Creates a new HopperSubsystem.
   */
  // Create a new SPARK MAX Controller for motor 1
  private static CANSparkMax motor1 = new CANSparkMax(Constants.Elevator1,MotorType.kBrushless);
  // Create a new SPARK MAX Controller for motor 2
  private static CANSparkMax motor2 = new CANSparkMax(Constants.Elevator2, MotorType.kBrushless);
  // Create a new integrated PID controller object
  private CANPIDController pidController;
  // Create a new encoder object
  private CANEncoder encoder;
  private boolean readyUse = false;
  private double position = 0;
  public double kP= .01, kI=0, kD=0, kIz, kFF,
  kMaxOutput = 1, kMinOutput = -1;
  private static final Logger LOGGER = Logger.getLogger(DriveSubsystem.class.getName());
  // Set Double Solenoid with:
  // Forward channel = 4
  // Backward channel = 7
  private DoubleSolenoid brakeElevator = new DoubleSolenoid(4,7);

  public ElevatorSubystem() {
    // Set brake elevator solenoid to backward
    brakeElevator.set(DoubleSolenoid.Value.kForward);

    // MOTOR 1
    // Restore motor1 controller parameters to factory default
    if(motor1.restoreFactoryDefaults() != CANError.kOk) {
      SmartDashboard.putString("Failed to reset Elevator motor 1 to Factory default", "Error");
    }
    //Sets the elevator motor 1 current limit in Amps at 30A
    if(motor1.setSmartCurrentLimit(40) != CANError.kOk) {
      SmartDashboard.putString("Failed to reset Elevator motor 1 smart current limit to 40A", "Error");
    }

    // MOTOR 2
    // Restore motor1 controller parameters to factory default
    if(motor2.restoreFactoryDefaults() != CANError.kOk) {
      SmartDashboard.putString("Failed to reset Elevator motor 2 to Factory default", "Error");
    }
    // Causes Motor 2 controller's output to mirror the Motor 1 (leader). 
    // Only voltage output is mirrored.
    if(motor2.follow(motor1, true) != CANError.kOk) {
      SmartDashboard.putString("Failed to mirroe Motor 2 with Motor 1.", "Error");
    }
    //Sets the elevator motor 1 current limit in Amps at 30A
    if(motor2.setSmartCurrentLimit(40) != CANError.kOk) {
      SmartDashboard.putString("Failed to set Elevator motor 2 smart current limit to 40A", "Error");
    }

    // Get an encoder object for motor 1
    encoder = motor1.getEncoder();
    // Set the encoder position to zero
    encoder.setPosition(0);

    // Get the integrated PID controller object for motor 1
    pidController = motor1.getPIDController();
    // Set the Proportional Gain constant of the PIDF controller to kP
    pidController.setP(kP);
    // Set the Integral Gain constant of the PIDF controller to kI
    pidController.setI(kI);
    // Set the Derivative Gain constant of the PIDF controller to kD
    pidController.setD(kD);
    // Set the IZone range of the PIDF controller to kIz
    pidController.setIZone(kIz);
    // Set the Feed-froward Gain constant of the PIDF controller to kFF
    pidController.setFF(kFF);
    // Set the min and max output for the closed loop mode to kMinOutput and kMaxOutput
    // min Reverse power minimum to allow the controller to output = kMinOutput
    // max Forward power maximum to allow the controller to output = kMaxOutput
    if(pidController.setOutputRange(kMinOutput, kMaxOutput) != CANError.kOk) {
      SmartDashboard.putString("Failed to set the elavator integrated PID controller closed loop mode output range", "Error");
    }
  }

  
  /** 
   * Move Elevator based on joystick input
   * @param joystickInput
   */
  public void moveLift(double joystickInput){
    if(readyUse == true){
      if((joystickInput < 0 && position > -570) || (joystickInput > 0 && position < 0))
        position = position+(10*joystickInput);
      if(position >=-570 && position <= 0)
        pidController.setReference(position, ControlType.kPosition);
      SmartDashboard.putNumber("commanded position", position);
    }
  }

  
  /**
   * Get encoder position object 
   * @return double
   */
  public double getPosition(){
      return encoder.getPosition();
  }

  /**
   * Set Elevator to ready
   */
  public void armElevator(){
    readyUse = true;
  }

  /**
   * Get Elevator status 
   * @return boolean
   */
  public boolean getReadyUse(){
    return readyUse;
  }

  /**
   * Set brake elevator solenoid to input state
   * @param state
   */
  public void setBrake(DoubleSolenoid.Value state){  
    // Set brake elevator solenoid to input state
    brakeElevator.set(state);
  }

  public void toggleBrake(){
    if(brakeElevator.get() == Value.kForward){
      brakeElevator.set(Value.kReverse);
    } else {
      brakeElevator.set(Value.kForward);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}