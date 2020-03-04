/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ConveyorSubsystem extends SubsystemBase {
    
  /**
   * Creates a new ExampleSubsystem.
   */
  private static CANSparkMax conveyor= new CANSparkMax(Constants.Belt, MotorType.kBrushless);

    public ConveyorSubsystem(){

        conveyor.restoreFactoryDefaults();
        conveyor.enableVoltageCompensation(12);
        conveyor.setIdleMode(IdleMode.kBrake);

  }
  public void runIntake(double speed){
        conveyor.set(speed);
    }

    public void stopIntake(){
        conveyor.set(0);
    }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
