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

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    
  /**
   * Creates a new ExampleSubsystem.
   */
  private static CANSparkMax intakeMotor= new CANSparkMax(Constants.Intake,MotorType.kBrushless);;
  private DoubleSolenoid intakeSolenoid2 = new DoubleSolenoid(0, 2);
  public enum IntakeDirection {IN, OUT}

    public IntakeSubsystem() {
        intakeSolenoid2.set(DoubleSolenoid.Value.kForward);

        intakeMotor.restoreFactoryDefaults();
        intakeMotor.enableVoltageCompensation(12);
        intakeMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.setSmartCurrentLimit(40);
        

  }
  public void runIntake(double speed){
        intakeMotor.set(speed);
    }

  public void setIntake(DoubleSolenoid.Value state){  
     intakeSolenoid2.set(state);

  }
  public void stopIntake(){
      intakeMotor.set(0);
  }

  public void toggleIntake(){
    if(intakeSolenoid2.get() == Value.kForward){
      intakeSolenoid2.set(Value.kReverse);
    }

    else{
      intakeSolenoid2.set(Value.kForward);
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
