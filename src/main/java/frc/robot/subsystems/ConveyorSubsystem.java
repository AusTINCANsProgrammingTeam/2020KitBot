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

public class ConveyorSubsystem extends SubsystemBase {
    
  /**
   * Creates a new ConveyorSubsystem.
   */
  // Create a new SPARK MAX Controller
  private static CANSparkMax conveyor= new CANSparkMax(Constants.Belt, MotorType.kBrushless);

    public ConveyorSubsystem(){

      // Restore motor controller parameters to factory default
      if(conveyor.restoreFactoryDefaults() != CANError.kOk) {
        SmartDashboard.putString("Failed to reset Conveyor motor Factory default", "Error");
      }
      // Sets the voltage compensation setting for all modes on the conveyor SPARK MAX
      // and enables voltage compensation with a 12 Volts Nominal voltage to compensate 
      // output to
      if(conveyor.enableVoltageCompensation(12) != CANError.kOk) {
        SmartDashboard.putString("Failed to enable Conveyor motor voltage compensation 12 Volts", "Error");
      }
      // Sets the idle mode setting for the conveyor SPARK MAX as brake.
      if(conveyor.setIdleMode(IdleMode.kBrake) != CANError.kOk) {
        SmartDashboard.putString("Failed to set Conveyor idle mode to Brake", "Error");
      }
      //Sets the current limit in Amps at 30A
      if(conveyor.setSmartCurrentLimit(30) != CANError.kOk) {
        SmartDashboard.putString("Failed to set Conveyor smart current limit to 30A", "Error");
      }

  }

  
  /** 
   * Run the intake motor and set speed
   * @param speed
   */
  public void runIntake(double speed){
    //Set the speed of the conveyor speed controller
    conveyor.set(speed);
  }

  public void stopIntake(){
    //Set the speed of the conveyor speed controller to zero thus stopping it
    conveyor.set(0);
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double conveyorTempC = conveyor.getMotorTemperature();
    double conveyorTempF = 9.0/5.0 * conveyorTempC + 32;
    SmartDashboard.putString("Conveyor motor temperature: "+ conveyorTempF + "°F", "Information");
  }

}
