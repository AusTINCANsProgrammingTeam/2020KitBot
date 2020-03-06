/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.logging.Logger;

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
    public static DoubleSolenoid hoodedShooter = new DoubleSolenoid(1,3);
    private double kP=.0003,kI=0.0,kD=0,kIz=0,kFF=0.000195,kMinOutput=0,kMaxOutput=1;
    private static final Logger LOGGER = Logger.getLogger(ShooterSubsystem.class.getName());

  /**
   * Creates a new ExampleSubsystem.
   */
  public ShooterSubsystem() {
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
    shooterMotor.setSmartCurrentLimit(30);
    shooterMotor.setInverted(true);
    hoodedShooter.set(DoubleSolenoid.Value.kForward);
    SmartDashboard.putNumber("Shooter P", kP);
    SmartDashboard.putNumber("Shooter I", kI);
    SmartDashboard.putNumber("Shooter D", kD);
    SmartDashboard.putNumber("Shooter FF", kFF);
  
  }

  public void setVelocitySetpoint(double setpoint){
    shooterPidController.setReference(setpoint, ControlType.kVelocity);
    // if(hoodedShooter.get() == Value.kForward && (getVelocity() >= 2350 && getVelocity() <= 2550))
    //   shooterReady = true;
    // else if(hoodedShooter.get() == Value.kReverse && (getVelocity() >= 3877 && getVelocity() <= 4077))
    // shooterReady = true;
    // else if(getVelocity() >= setpoint -100 && getVelocity() <= setpoint+100)
    //   shooterReady = true;
    // else
    //   shooterReady = false;
  }

  public void armShooter(double targetSpeed){
    if( (getVelocity() >= targetSpeed-100 && getVelocity() <= targetSpeed+100))
      shooterReady = true;
    else
      shooterReady = false;
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
    SmartDashboard.putNumber("Shooter Speed", shooterEncoder.getVelocity());

    double p = SmartDashboard.getNumber("Shooter P", 0);
    double i = SmartDashboard.getNumber("Shooter I", 0);
    double d = SmartDashboard.getNumber("Shooter D", 0);
    double ff = SmartDashboard.getNumber("Shooter FF", 0);


    if((p != kP)) { shooterPidController.setP(p); kP = p;
    LOGGER.warning("pee pee poo poo");} 
      if((i != kI)) { shooterPidController.setI(i); kI = i;
        LOGGER.warning("iee iee ioo ioo");} 
      if((d != kD)) { shooterPidController.setD(d); kD = d;
        LOGGER.warning("dee dee doo doo");} 
        if((ff != kFF)) { shooterPidController.setFF(ff); kFF = ff;
          LOGGER.warning("fee fee foo foo");} 
    
    // This method will be called once per scheduler run
  }
}
