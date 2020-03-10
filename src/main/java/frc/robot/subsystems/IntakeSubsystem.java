/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    
  /**
   * Creates a new IntakeSubsystem.
   */
  // Create a new SPARK MAX Controller for intake motor
  private static CANSparkMax intakeMotor= new CANSparkMax(Constants.Intake,MotorType.kBrushless);

  // Set Double Solenoid with:
  // Forward channel = 0
  // Backward channel = 2
  private DoubleSolenoid intakeSolenoid2 = new DoubleSolenoid(0, 2);
  public enum IntakeDirection {IN, OUT}

  public IntakeSubsystem() {
    // Set brake intake solenoid to backward
    intakeSolenoid2.set(DoubleSolenoid.Value.kForward);

    // Restore intake motor controller parameters to factory default
    if(intakeMotor.restoreFactoryDefaults() != CANError.kOk) {
      SmartDashboard.putString("Failed to reset Intake motor to Factory default", "Error");
    }
    // Sets the voltage compensation setting for all modes on the intake motor SPARK MAX
    // and enables voltage compensation with a 12 Volts Nominal voltage to compensate 
    // output to
    if(intakeMotor.enableVoltageCompensation(12) != CANError.kOk) {
      SmartDashboard.putString("Failed to enable Intake motor voltage compensation to 12 Volts", "Error");
    }
    // Sets the idle mode setting for the intake SPARK MAX as brake.
    if(intakeMotor.setIdleMode(IdleMode.kBrake) != CANError.kOk) {
      SmartDashboard.putString("Failed to set intake motor idle mode to Brake", "Error");
    }
    // Sets the current limit in Amps for intake motor at 40A
    if(intakeMotor.setSmartCurrentLimit(40) != CANError.kOk) {
      SmartDashboard.putString("Failed to set intake motor smart current limit to 40A", "Error");
    }
  }

  
  /** 
   * Set the speed of the intake motor controller
   * @param speed
   */
  public void runIntake(double speed){
    // Set the speed of the intake motor controller
    intakeMotor.set(speed);
  }

  
  /** 
   * Set state of the intake solenoid
   * @param state
   */
  public void setIntake(DoubleSolenoid.Value state){  
    // Set state of the intake solenoid
    intakeSolenoid2.set(state);

  }

  /**
   * Stop intake by setting speed to zero
   */
  public void stopIntake(){
    intakeMotor.set(0);
  }

  /**
   * Toggle intake solenoid
   */
  public void toggleIntake(){
    if(intakeSolenoid2.get() == Value.kForward){
      intakeSolenoid2.set(Value.kReverse);
    } else {
      intakeSolenoid2.set(Value.kForward);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}