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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HopperSubsystem extends SubsystemBase {
    
  /**
   * Creates a new HopperSubsystem.
   */
  // Create a new SPARK MAX Controller for hopper motor 1
  private static CANSparkMax hopper1= new CANSparkMax(Constants.Hopper1, MotorType.kBrushless);
  // Create a new SPARK MAX Controller for hopper motor 2
  private static CANSparkMax hopper2= new CANSparkMax(Constants.Hopper2, MotorType.kBrushless);

  public HopperSubsystem() {
    // Restore hopper1 motor controller parameters to factory default
    if(hopper1.restoreFactoryDefaults() != CANError.kOk) {
      SmartDashboard.putString("Failed to reset Hopper motor 1 to Factory default", "Error");
    }
    // Sets the voltage compensation setting for all modes on the hopper motor 1 SPARK MAX
    // and enables voltage compensation with a 12 Volts Nominal voltage to compensate 
    // output to
    if(hopper1.enableVoltageCompensation(12) != CANError.kOk) {
      SmartDashboard.putString("Failed to enable Hopper motor 1 voltage compensation to 12 Volts", "Error");
    }
    // Sets the idle mode setting for the conveyor SPARK MAX as brake.
    if(hopper1.setIdleMode(IdleMode.kBrake) != CANError.kOk) {
      SmartDashboard.putString("Failed to set Hopper motor 1 idle mode to Brake", "Error");
    }
    // Restore hopper2 motor controller parameters to factory default
    if(hopper2.restoreFactoryDefaults() != CANError.kOk) {
      SmartDashboard.putString("Failed to reset Hopper motor 2 to Factory default", "Error");
    }
    // Sets the voltage compensation setting for all modes on the hopper motor 2 SPARK MAX
    // and enables voltage compensation with a 12 Volts Nominal voltage to compensate 
    // output to
    if(hopper2.enableVoltageCompensation(12) != CANError.kOk) {
      SmartDashboard.putString("Failed to enable Hopper motor 2 voltage compensation to 12 Volts", "Error");
    }
    // Sets the idle mode setting for the conveyor SPARK MAX as brake.
    if(hopper2.setIdleMode(IdleMode.kBrake) != CANError.kOk) {
      SmartDashboard.putString("Failed to set Hopper motor 2 idle mode to Brake", "Error");
    }
    // Set inverting direction of a hopper motor 1 speed controller
    hopper1.setInverted(true);
    // Set inverting direction of a hopper motor 2 speed controller
    hopper2.setInverted(true);
    // Sets the current limit in Amps for hopper motor 1 at 30A
    if(hopper1.setSmartCurrentLimit(30) != CANError.kOk) {
      SmartDashboard.putString("Failed to set Hopper motor 1 smart current limit to 30A", "Error");
    }
    // Sets the current limit in Amps for hopper motor 2 at 30A
    if(hopper2.setSmartCurrentLimit(30) != CANError.kOk) {
      SmartDashboard.putString("Failed to set Hopper motor 2 smart current limit to 30A", "Error");
    }
  }

  
  /** 
   * Run Intake with a set of defimed speeds for motor 1 and 2
   * @param speed1
   * @param speed2
   */
  public void runIntake(double speed1, double speed2){
    // Set the speed of both hopper motor speed controller
    // Motor 1
    hopper1.set(speed1);
    // Motor 2
    hopper2.set(speed2);
  }

  /**
   * Stop intake motors by setting speeds to zero
   */
  public void stopIntake(){
    // Set the speed of both hopper motor speed controller to zero
    // thus stopping both motors
    hopper1.set(0);
    hopper2.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}