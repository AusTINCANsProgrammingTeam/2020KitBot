/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private CANPIDController shooterPidController;
    private static CANSparkMax shooterMotor;   
    private CANEncoder shooterEncoder;
    public static boolean shooterReady = false;
    private DoubleSolenoid hoodedShooter = new DoubleSolenoid(1,3);
    private double kP=3e-4,kI=1e-6,kD=0,kIz=0,kFF=-0,kMinOutput=0,kMaxOutput=1;
  /**
   * Creates a new ExampleSubsystem.
   */
  public ShooterSubsystem() {
    SmartDashboard.putNumber("shooter speed", .45);
    shooterMotor = new CANSparkMax(Constants.Shooter, MotorType.kBrushless);
    shooterMotor.restoreFactoryDefaults();
    shooterMotor.setIdleMode(IdleMode.kCoast);
    shooterPidController = shooterMotor.getPIDController();
    shooterEncoder = shooterMotor.getEncoder();
    shooterPidController.setP(kP);
    shooterPidController.setI(kI);
    shooterPidController.setD(kD);
    shooterPidController.setIZone(kIz);
    shooterPidController.setFF(kFF);
    shooterPidController.setOutputRange(kMinOutput, kMaxOutput);
    shooterMotor.setInverted(true);
    hoodedShooter.set(DoubleSolenoid.Value.kForward);
  
  }

  public void setVelocitySetpoint(double setpoint){
    shooterPidController.setReference(setpoint, ControlType.kVelocity);
    if(getVelocity() >= setpoint)
      shooterReady = true;
  }

  public void setSpeed(double setpoint){
    shooterMotor.set(setpoint);
  }


  public double getVelocity(){
      return shooterEncoder.getVelocity();
  }
  
  public void toggleHood(){
    if(hoodedShooter.get() == Value.kForward){
      hoodedShooter.set(Value.kReverse);
    }

    else{
      hoodedShooter.set(Value.kForward);
    }
   
  }

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
  }
}
