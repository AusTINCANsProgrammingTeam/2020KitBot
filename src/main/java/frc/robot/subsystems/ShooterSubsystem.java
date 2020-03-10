/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.logging.Logger;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;
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
    // Create and Set Double Solenoid with:
    // Forward channel = 1
    // Backward channel = 3
    public static DoubleSolenoid hoodedShooter = new DoubleSolenoid(1,3);
    private double kP=.0003,kI=0.0,kD=0,kIz=0,kFF=0.000195,kMinOutput=0,kMaxOutput=1;
    private static final Logger LOGGER = Logger.getLogger(ShooterSubsystem.class.getName());

  /**
   * Creates a new ShooterSubsystem.
   */
  public ShooterSubsystem() {
    // Create a new SPARK MAX Controller for shooter motor
    shooterMotor = new CANSparkMax(Constants.Shooter, MotorType.kBrushless);
    // Restore intake motor controller parameters to factory default
    if(shooterMotor.restoreFactoryDefaults() != CANError.kOk) {
      SmartDashboard.putString("Failed to reset shooter motor to Factory default", "Error");
    }
    // Sets the idle mode setting for the shooter SPARK MAX as brake.
    if(shooterMotor.setIdleMode(IdleMode.kCoast) != CANError.kOk) {
      SmartDashboard.putString("Failed to set shooter motor idle mode to Brake", "Error");
    }
    // Get the integrated PID controller object for shooter motor
    shooterPidController = shooterMotor.getPIDController();
    // Get an encoder object for shooter motor
    shooterEncoder = shooterMotor.getEncoder();
    // Set the Proportional Gain constant of the PIDF controller on the SPARK MAX to kP
    if(shooterPidController.setP(kP) != CANError.kOk) {
      SmartDashboard.putString("Failed to set shooter motor Proportional Gain constant", "Error");
    }
    // Set the Integral Gain constant of the PIDF controller on the SPARK MAX to kI
    if(shooterPidController.setI(kI) != CANError.kOk) {
      SmartDashboard.putString("Failed to set shooter motor Integral Gain constatn", "Error");
    }
    // Set the Derivative Gain constant of the PIDF controller on the SPARK MAX to kD
    if(shooterPidController.setD(kD) != CANError.kOk) {
      SmartDashboard.putString("Failed to set shooter motor Derivative Gain constant", "Error");
    }
    // Set the IZone range of the PIDF controller on the SPARK MAX to kIz
    if(shooterPidController.setIZone(kIz) != CANError.kOk) {
      SmartDashboard.putString("Failed to set shooter motor IZone range of the PIDF controller", "Error");
    }
    // Set the Feed-froward Gain constant of the PIDF controller on the SPARK MAX to kFF
    if(shooterPidController.setFF(kFF) != CANError.kOk) {
      SmartDashboard.putString("Failed to set shooter motor Feed-froward Gain constant", "Error");
    }
    // Set the min and max output for the closed loop mode. 
    if(shooterPidController.setOutputRange(kMinOutput, kMaxOutput) != CANError.kOk) {
      SmartDashboard.putString("Failed to set shooter motor min and max output for the closed loop mode", "Error");
    }
    // Sets the current limit in Amps for shooter motor at 30A
    if(shooterMotor.setSmartCurrentLimit(30) != CANError.kOk) {
      SmartDashboard.putString("Failed to set shooter motor smart current limit to 30A", "Error");
    }
    // Set inverting direction of a shooter motor speed controller
    shooterMotor.setInverted(true);
    // Set the value of the shooter solenoid to forward
    hoodedShooter.set(DoubleSolenoid.Value.kForward);
    // Put numbers in table
    SmartDashboard.putNumber("Shooter P", kP);
    SmartDashboard.putNumber("Shooter I", kI);
    SmartDashboard.putNumber("Shooter D", kD);
    SmartDashboard.putNumber("Shooter FF", kFF);
  }

  /** 
   * Set the controller reference value based on the selected control mode
   * @param setpoint
   */
  public void setVelocitySetpoint(double setpoint){
    // Set the controller reference value based on the selected control mode
    if(shooterPidController.setReference(setpoint, ControlType.kVelocity) != CANError.kOk) {
      SmartDashboard.putString("Failed to set shooter motor controller reference value based on the selected control mode", "Error");
    }
    // Put shooter set point in table
    SmartDashboard.putNumber("ShooterSetpoint", setpoint);
    // if(hoodedShooter.get() == Value.kForward && (getVelocity() >= 2350 && getVelocity() <= 2550))
    //   shooterReady = true;
    // else if(hoodedShooter.get() == Value.kReverse && (getVelocity() >= 3877 && getVelocity() <= 4077))
    // shooterReady = true;
    // else if(getVelocity() >= setpoint -100 && getVelocity() <= setpoint+100)
    //   shooterReady = true;
    // else
    //   shooterReady = false;
  }

  /** 
   * Arm shooter and set speed
   * @param targetSpeed
   */
  public void armShooter(double targetSpeed){
    if( (getVelocity() >= targetSpeed-100 && getVelocity() <= targetSpeed+100))
      shooterReady = true;
    else
      shooterReady = false;
  }
  
  /** 
   * Set speed on speed controller
   * @param setpoint
   */
  public void setSpeed(double setpoint){
    // Set speed on speed controller
    shooterMotor.set(setpoint);
    // Put speed in table
    SmartDashboard.putNumber("ShooterSetpoint", setpoint);
  }

  /** 
   * @return double
   */
  public double getVelocity(){
      return shooterEncoder.getVelocity();
  }
  
  /**
   * Toggle Hood
   */
  public void toggleHood(){
    // Get the velocity of the motor
    if(hoodedShooter.get() == Value.kForward){
      // Reverse solenoid
      hoodedShooter.set(Value.kReverse);
    } else{
      // Solenoid forward
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

    // Not sure how informative this is!
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