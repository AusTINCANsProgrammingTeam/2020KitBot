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
  private CANSparkMax intakeMotor= new CANSparkMax(11, MotorType.kBrushless);;
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
            break;
        case kForward:
            setIntake(DoubleSolenoid.Value.kReverse);
            break;
        case kOff:
            break;
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}