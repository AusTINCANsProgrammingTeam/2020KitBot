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

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private CANPIDController shooterPidController;
    private CANSparkMax shooterMotor;   
    private CANEncoder shooterEncoder;
    private double kP=3e-4,kI=1e-6,kD=0,kIz=0,kFF=-0,kMinOutput=-1,kMaxOutput=1;
  /**
   * Creates a new ExampleSubsystem.
   */
  public ShooterSubsystem() {
    shooterMotor = new CANSparkMax(5, MotorType.kBrushless);
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

  }

  public void setVelocitySetpoint(double setpoint){
    shooterPidController.setReference(setpoint, ControlType.kVelocity);
  }

  public double shooterVelocity(){
      return shooterEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
