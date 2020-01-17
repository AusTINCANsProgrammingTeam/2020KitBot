/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.DriveCommand;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANError;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import java.util.logging.*;


public class DriveSubsystem extends SubsystemBase {
    private CANSparkMax mLeft1;
    private CANSparkMax mLeft2;
    private CANSparkMax mRight1;
    private CANSparkMax mRight2;
    private CANPIDController l_pidController;
    private CANPIDController r_pidController;
    private CANEncoder l_encoder;
    private CANEncoder r_encoder;
    private DifferentialDrive differentialDrive;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
    private static final Logger LOGGER = Logger.getLogger(DriveSubsystem.class.getName());

  public DriveSubsystem() {
    mLeft1 = new CANSparkMax(1, MotorType.kBrushless);
    mLeft2 = new CANSparkMax(2, MotorType.kBrushless);
    mRight1 = new CANSparkMax(3, MotorType.kBrushless);
    mRight2 = new CANSparkMax(4, MotorType.kBrushless);
    mLeft1.restoreFactoryDefaults();
    mLeft2.restoreFactoryDefaults();
    mRight1.restoreFactoryDefaults();
    mRight2.restoreFactoryDefaults();
    mLeft1.enableVoltageCompensation(12);
    mLeft2.enableVoltageCompensation(12);
    mRight1.enableVoltageCompensation(12);
    mRight2.enableVoltageCompensation(12);
    mLeft1.setIdleMode(IdleMode.kBrake);
    mLeft2.setIdleMode(IdleMode.kBrake);
    mRight1.setIdleMode(IdleMode.kBrake);
    mRight2.setIdleMode(IdleMode.kBrake);
    mLeft2.follow(mLeft1);
    mRight2.follow(mRight1);
    differentialDrive = new DifferentialDrive(mLeft1, mRight1);


    l_pidController = mLeft1.getPIDController();
    r_pidController = mRight1.getPIDController();

    l_encoder = mLeft1.getEncoder();
    r_encoder = mLeft1.getEncoder();

    l_pidController.setP(Constants.kP);
    l_pidController.setI(Constants.kI);
    l_pidController.setD(Constants.kD);
    l_pidController.setIZone(Constants.kIz);
    l_pidController.setFF(Constants.kFF);
    l_pidController.setOutputRange(Constants.kMinOutput, Constants.kMaxOutput);
    r_pidController.setP(Constants.kP);
    r_pidController.setI(Constants.kI);
    r_pidController.setD(Constants.kD);
    r_pidController.setIZone(Constants.kIz);
    r_pidController.setFF(Constants.kFF);
    r_pidController.setOutputRange(Constants.kMinOutput, Constants.kMaxOutput);
    SmartDashboard.putNumber("P Gain", Constants.kP);
    SmartDashboard.putNumber("I Gain", Constants.kI);
    SmartDashboard.putNumber("D Gain", Constants.kD);
    SmartDashboard.putNumber("FF Value", Constants.kFF);
    differentialDrive.setSafetyEnabled(false);
  }

  public void arcadeDrive(double velocity, double heading){
    this.differentialDrive.arcadeDrive(velocity, heading, true);
  }

  public void tankDrive(double lVelocity, double rVelocity){
    this.differentialDrive.tankDrive(lVelocity, rVelocity);
  }

  public void updatePID(){
  double p = SmartDashboard.getNumber("P Gain", 0);
  double i = SmartDashboard.getNumber("I Gain", 0);
  double d = SmartDashboard.getNumber("D Gain", 0);
  // double iz = SmartDashboard.getNumber("I Zone", 0);
  // double ff = SmartDashboard.getNumber("Feed Forward", 0);
  double max = SmartDashboard.getNumber("Max Output", 0);
  double min = SmartDashboard.getNumber("Min Output", 0);

  //if PID coefficients on SmartDashboard have changed, write new values to controller
  if((p != kP)) { l_pidController.setP(p); r_pidController.setP(p); kP = p; 
  LOGGER.warning("PID CHANGED");}
  if((i != kI)) { l_pidController.setI(i); r_pidController.setI(i); kI = i; 
  LOGGER.warning("PID CHANGED");}
  if((d != kD)) { l_pidController.setD(d); r_pidController.setD(d); kD = d; 
  LOGGER.warning(l_pidController.getD() +" D CHANGED");}
  // if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
  // if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
  if((max != kMaxOutput) || (min != kMinOutput)) 
  { 
      // m_pidController.setOutputRange(min, max); 
      // kMinOutput = min;
      // kMaxOutput = max; 
  }
}
public void setLeftPidVelocitySetpoint(double setpoint)
{
    l_pidController.setReference(setpoint, ControlType.kVelocity);
}

public void setRightPidVelocitySetpoint(double setpoint)
{
    r_pidController.setReference(setpoint, ControlType.kVelocity);
}

public double leftVelocity()
{
    return l_encoder.getVelocity();
}

public double rightVelocity()
{
    return r_encoder.getVelocity();
}

public double fpsToRPM(double fps){
    fps = fps * 12;
    fps = fps/Constants.kWheelCircumference;
    fps = fps *60;
    fps = fps*Constants.kGearRatio;
    return fps;
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   new DriveCommand();
  }
}

