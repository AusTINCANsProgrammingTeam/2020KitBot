package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HopperSubsystem extends SubsystemBase {
    
  /**
   * Creates a new ExampleSubsystem.
   */
  private CANSparkMax hopper1= new CANSparkMax(Constants.Hopper1, MotorType.kBrushless);
  private CANSparkMax hopper2= new CANSparkMax(Constants.Hopper2, MotorType.kBrushless);;

    public HopperSubsystem(){
        hopper2.follow(hopper1);
        hopper1.restoreFactoryDefaults();
        hopper1.enableVoltageCompensation(12);
        hopper1.setIdleMode(IdleMode.kBrake);
        hopper2.restoreFactoryDefaults();
        hopper2.enableVoltageCompensation(12);
        hopper2.setIdleMode(IdleMode.kBrake);

  }
  public void runIntake(double speed){
        hopper1.set(speed);
    }

    public void stopIntake(){
        hopper1.set(0);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}