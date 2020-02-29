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
import frc.robot.RobotContainer;
import frc.robot.commands.DriveCommand;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import java.util.logging.*;


public class DriveSubsystem extends SubsystemBase {
    private static CANSparkMax mLeft1 = new CANSparkMax(Constants.DriveLeft1, MotorType.kBrushless);
    private static CANSparkMax mLeft2 = new CANSparkMax(Constants.DriveLeft2, MotorType.kBrushless);
    private static CANSparkMax mLeft3 = new CANSparkMax(Constants.DriveLeft3, MotorType.kBrushless);
    private static CANSparkMax mRight1 = new CANSparkMax(Constants.DriveRight1, MotorType.kBrushless);
    private static CANSparkMax mRight2 = new CANSparkMax(Constants.DriveRight2, MotorType.kBrushless);
    private static CANSparkMax mRight3 = new CANSparkMax(Constants.DriveRight3, MotorType.kBrushless);
    private CANPIDController l_pidController;
    private CANPIDController r_pidController;
    private CANEncoder l_encoder;
    private CANEncoder r_encoder;
    private DifferentialDrive differentialDrive;
    public double kP = Constants.kPDrive, kI = Constants.kIDrive, kD = Constants.kDDrive, kIz = Constants.kIzDrive, kFF = Constants.kFFDrive,
     kMaxOutput = Constants.kMaxOutputDrive, kMinOutput = Constants.kMinOutputDrive;
    private static final Logger LOGGER = Logger.getLogger(DriveSubsystem.class.getName());

  public DriveSubsystem() {
    intializeDriveSubystem(mLeft1, mLeft2, mLeft3);
    intializeDriveSubystem(mRight1, mRight2, mRight3);
    differentialDrive = new DifferentialDrive(mLeft1, mRight1);


    l_pidController = mLeft1.getPIDController();
    r_pidController = mRight1.getPIDController();

    l_encoder = mLeft1.getEncoder();
    r_encoder = mRight1.getEncoder();
    l_encoder.setPosition(0);
    r_encoder.setPosition(0);

    l_pidController.setP(kP);
    l_pidController.setI(kI);
    l_pidController.setD(kD);
    l_pidController.setIZone(kIz);
    l_pidController.setFF(kFF);
    l_pidController.setOutputRange(kMinOutput, kMaxOutput);
    r_pidController.setP(kP);
    r_pidController.setI(kI);
    r_pidController.setD(kD);
    r_pidController.setIZone(kIz);
    r_pidController.setFF(kFF);
    r_pidController.setOutputRange(kMinOutput, kMaxOutput);
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("FF Value", kFF);
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

public void setLeftSetpoint(double setpoint)
{
    mLeft1.set(setpoint);
}

public void setRightSetpoint(double setpoint)
{
    mRight1.set(setpoint);
}

public double leftVelocity()
{
    return l_encoder.getVelocity();
}

public double rightVelocity()
{
    return r_encoder.getVelocity();
}

public double getLeftEncPosition()
{
  return l_encoder.getPosition();
}

public double getRightEncPosition()
{
  return r_encoder.getPosition();
}

public double fpsToRPM(double fps){
    fps = fps * 12;
    fps = fps/Constants.kWheelCircumference;
    fps = fps *60;
    fps = fps*Constants.kGearRatio;
    return fps;
}

public void intializeDriveSubystem(CANSparkMax master, CANSparkMax... slaves){
  master.restoreFactoryDefaults();
  master.enableVoltageCompensation(12);
  master.setIdleMode(IdleMode.kBrake);
  master.setOpenLoopRampRate(.2);
  master.setSmartCurrentLimit(40);
 
  for(CANSparkMax slave : slaves) {
    slave.restoreFactoryDefaults();
    slave.enableVoltageCompensation(12);
    slave.setIdleMode(IdleMode.kBrake);
    slave.setOpenLoopRampRate(.2);
    slave.setSmartCurrentLimit(40);
    slave.follow(master);
}
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Velocity", -1 * RobotContainer.mDriveSubsystem.leftVelocity());    
    SmartDashboard.putNumber("Right Velocity", -1 * RobotContainer.mDriveSubsystem.rightVelocity());
    SmartDashboard.putNumber("lift position", RobotContainer.mElevatorSubystem.getPosition());
    SmartDashboard.putNumber("left encoder", RobotContainer.mDriveSubsystem.getLeftEncPosition());
    SmartDashboard.putNumber("right encoder", RobotContainer.mDriveSubsystem.getRightEncPosition());
    // This method will be called once per scheduler run
  }
}

