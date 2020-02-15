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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    
  /**
   * Creates a new ExampleSubsystem.
   */
  private CANSparkMax intakeMotor= new CANSparkMax(6, MotorType.kBrushless);;
  private DoubleSolenoid intakeSolenoid = new DoubleSolenoid(1, 2);
  public enum IntakeDirection {IN, OUT}

    public IntakeSubsystem() {
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.enableVoltageCompensation(12);
        intakeMotor.setIdleMode(IdleMode.kBrake);

  }
  public void runIntake(double speed){
        intakeMotor.set(speed);
    }

  public void setIntake(DoubleSolenoid.Value state){  
      intakeSolenoid.set(state);
  }
  public void stopIntake(){
      intakeMotor.set(0);
  }

  public void toggleIntake(){
    switch(intakeSolenoid.get()){
        case kReverse:
        setIntake(DoubleSolenoid.Value.kForward);
        case kForward:
        setIntake(DoubleSolenoid.Value.kReverse);
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
