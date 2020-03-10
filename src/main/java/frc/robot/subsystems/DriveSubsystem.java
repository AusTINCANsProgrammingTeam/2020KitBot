/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import java.util.logging.Logger;


public class DriveSubsystem extends SubsystemBase {
    // Create new SPARK MAX Controller objects
    private static CANSparkMax mLeft1 = new CANSparkMax(Constants.DriveLeft1, MotorType.kBrushless);
    private static CANSparkMax mLeft2 = new CANSparkMax(Constants.DriveLeft2, MotorType.kBrushless);
    private static CANSparkMax mLeft3 = new CANSparkMax(Constants.DriveLeft3, MotorType.kBrushless);
    private static CANSparkMax mRight1 = new CANSparkMax(Constants.DriveRight1, MotorType.kBrushless);
    private static CANSparkMax mRight2 = new CANSparkMax(Constants.DriveRight2, MotorType.kBrushless);
    private static CANSparkMax mRight3 = new CANSparkMax(Constants.DriveRight3, MotorType.kBrushless);
    // Create Integrated PID Controller objects
    private CANPIDController l_pidController;
    private CANPIDController r_pidController;
    // Create encoder objects
    public static CANEncoder l_encoder;
    public static CANEncoder r_encoder;
    // Create a driving differential drive/skid-steer object
    private DifferentialDrive differentialDrive;
    public double kP = Constants.kPDrive, kI = Constants.kIDrive, kD = Constants.kDDrive, kIz = Constants.kIzDrive, kFF = Constants.kFFDrive,
     kMaxOutput = Constants.kMaxOutputDrive, kMinOutput = Constants.kMinOutputDrive;

    public double kPTurn = 0, kITurn = Constants.kIDrive, kDTurn = Constants.kDDrive, kIzTurn = Constants.kIzDrive, kFFTurn = Constants.kFFDrive,
     kMaxOutputTurn = Constants.kMaxOutputDrive, kMinOutputTurn = Constants.kMinOutputDrive;
    private static final Logger LOGGER = Logger.getLogger(DriveSubsystem.class.getName());
    int slaveDeviceID;

  public DriveSubsystem() {
    intializeDriveSubystem(mLeft1, mLeft2, mLeft3);
    intializeDriveSubystem(mRight1, mRight2, mRight3);
    differentialDrive = new DifferentialDrive(mLeft1, mRight1);

    // Get Integrated PID Controller object for motor left 1
    l_pidController = mLeft1.getPIDController();
    // Get Integrated PID Controller object for motor right 1
    r_pidController = mRight1.getPIDController();

    // Get encoder object for motor left 1
    l_encoder = mLeft1.getEncoder();
    // Get encoder object for motor right 1
    r_encoder = mRight1.getEncoder();
    // Set the position of the encoder for left motor to zero
    l_encoder.setPosition(0);
    // Set the position of the encoder for right motor to zero
    r_encoder.setPosition(0);

    // Set the Proportional Gain constant of the left motor PIDF controller on the SPARK MAX to kP
    l_pidController.setP(kP);
    // Set the Integral Gain constant of the left motor PIDF controller on the SPARK MAX to kI
    l_pidController.setI(kI);
    // Set the Derivative Gain constant of the left motor PIDF controller on the SPARK MAX to kD
    l_pidController.setD(kD);
    // Set the IZone range of the left motor PIDF controller on the SPARK MAX to kIz
    l_pidController.setIZone(kIz);
    // Set the Feed-froward Gain constant of the left motor PIDF controller on the SPARK MAX to kFF
    l_pidController.setFF(kFF);
    // Set the min amd max output for the closed loop mode for left motor
    l_pidController.setOutputRange(kMinOutput, kMaxOutput);
    // Set the Proportional Gain constant of the right motor PIDF controller on the SPARK MAX to kP
    r_pidController.setP(kP);
    // Set the Integral Gain constant of the right motor PIDF controller on the SPARK MAX to kI
    r_pidController.setI(kI);
    // Set the Derivative Gain constant of the right motor PIDF controller on the SPARK MAX to kD
    r_pidController.setD(kD);
    // Set the IZone range of the right motor PIDF controller on the SPARK MAX to kIz
    r_pidController.setIZone(kIz);
    //  Set the Feed-froward Gain constant of the right motor PIDF controller on the SPARK MAX to kFF
    r_pidController.setFF(kFF);
    // Set the min amd max output for the closed loop mode for right motor
    r_pidController.setOutputRange(kMinOutput, kMaxOutput);
    // Put these numbers in table
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("FF Value", kFF);
    differentialDrive.setSafetyEnabled(false);
  }

  
  /** 
   * Arcade drive method for differential drive platform.
   * @param velocity
   * @param heading
   */
  public void arcadeDrive(double velocity, double heading){
    this.differentialDrive.arcadeDrive(velocity, heading, true);
  }

  
  /** 
   * Tank drive method for differential drive platform. 
   * The calculated values will be squared to decrease sensitivity at low speeds.
   * @param lVelocity
   * @param rVelocity
   */
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
    // if((p != kP)) { l_pidController.setP(p); r_pidController.setP(p); kP = p; 
    // LOGGER.warning("PID CHANGED");}
    // if((i != kI)) { l_pidController.setI(i); r_pidController.setI(i); kI = i; 
    // LOGGER.warning("PID CHANGED");}
    // if((d != kD)) { l_pidController.setD(d); r_pidController.setD(d); kD = d; 
    // LOGGER.warning(l_pidController.getD() +" D CHANGED");}
    // if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
    // if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) 
    { 
        // m_pidController.setOutputRange(min, max); 
        // kMinOutput = min;
        // kMaxOutput = max; 
    }
  }

  /** 
   * Set the controller reference value based on the selected control mode.
   * @param setpoint
   */
  public void setLeftPidVelocitySetpoint(double setpoint) {
      l_pidController.setReference(setpoint, ControlType.kVelocity);
  }

  public void checkPathEnd() {
  }

  /** 
   * Set the controller reference value based on the selected control mode.
   * @param setpoint
   */
  public void setRightPidVelocitySetpoint(double setpoint) {
      r_pidController.setReference(setpoint, ControlType.kVelocity);
  }

  /** 
   * Set the controller reference value based on the selected control mode.
   * @param setpoint
   */
  public void setLeftPidPositionSetpoint(double setpoint) {
      l_pidController.setReference(setpoint, ControlType.kPosition);
  }

  /** 
   * Set the controller reference value based on the selected control mode.
   * @param setpoint
   */
  public void setRightPidPositionSetpoint(double setpoint)  {
      r_pidController.setReference(setpoint, ControlType.kPosition);
  }

  /** 
   * Set the speed of a speed controller for left motor 1.
   * @param setpoint
   */
  public void setLeftSetpoint(double setpoint) {
      mLeft1.set(setpoint);
  }

  /** 
   * Set the speed of a speed controller for right motor 1.
   * @param setpoint
   */
  public void setRightSetpoint(double setpoint) {
      mRight1.set(setpoint);
  }

  /** 
   * Get the velocity of the left motor and return it to caller
   * @return double
   */
  public double leftVelocity() {
      return l_encoder.getVelocity();
  }

  /** 
   * Get the velocity of the right motor and return it to caller
   * @return double
   */
  public double rightVelocity() {
      return r_encoder.getVelocity();
  }

  /** 
   * Get the position of the left motor
   * @return double
   */
  public double getLeftEncPosition() {
    return l_encoder.getPosition();
  }

  /** 
   * Get the position of the right motor
   * @return double
   */
  public double getRightEncPosition() {
    return r_encoder.getPosition();
  }

  /** 
   * Calculate Revolutions Per Minute from Frames Per Second (Hertz)
   * One Revolution per minute is equal to 1/60 Hertz
   * @param fps
   * @return double
   */
  public double fpsToRPM(double fps){
      fps = fps * 12;
      fps = fps/Constants.kWheelCircumference;
      fps = fps *60;
      fps = fps*Constants.kGearRatio;
      return fps;
  }

  /** 
   * Initialize Drive Train
   * @param master
   * @param slaves
   */
  public void intializeDriveSubystem(CANSparkMax master, CANSparkMax... slaves){
    //  Restore master drive train motor controller parameters to factory default
    if(master.restoreFactoryDefaults() != CANError.kOk) {
      SmartDashboard.putString("Failed to reset master drive train motors to Factory default", "Error");
    }
    // Sets the voltage compensation setting for all modes on the hopper motor 2 SPARK MAX
    // and enables voltage compensation with a 12 Volts Nominal voltage to compensate 
    // output to
    if(master.enableVoltageCompensation(12) != CANError.kOk) {
      SmartDashboard.putString("Failed to enable master drive train motor voltage compensation to 12 Volts", "Error");
    }
    // Sets the idle mode setting for the conveyor SPARK MAX as brake.
    if(master.setIdleMode(IdleMode.kBrake) != CANError.kOk) {
      SmartDashboard.putString("Failed to set master drive train motors idle mode to Brake", "Error");
    }
    // Sets the ramp rate for open loop control modes for master drive train.
    if(master.setOpenLoopRampRate(.2) != CANError.kOk) {
      SmartDashboard.putString("Failed to set the ramp rate for open loop control modes for master drive train", "Error");
    }
    // Sets the current limit in Amps for master drive train motors at 40A
    if(master.setSmartCurrentLimit(40) != CANError.kOk) {
      SmartDashboard.putString("Failed to set the current limit in Amps for master drive train motors at 40A", "Error");
    }
  
    // For all slave motors, initialize them
    for(CANSparkMax slave : slaves) {
      // Get the configured Device ID of the slave SPARK MAX
      slaveDeviceID = slave.getDeviceId();
      String strSlaveDeviceID = String.valueOf(slaveDeviceID);
      // Restore slave motor controller parameters to factory default
      if(slave.restoreFactoryDefaults() != CANError.kOk) {
        SmartDashboard.putString("Failed to reset slave drive train motor with ID=" + strSlaveDeviceID + 
          " to Factory default", "Error");
      }
      // Sets the voltage compensation setting for all modes on the hopper motor 2 SPARK MAX
      // and enables voltage compensation with a 12 Volts Nominal voltage to compensate 
      // output to
      if(slave.enableVoltageCompensation(12) != CANError.kOk) {
        SmartDashboard.putString("Failed to enable slave drive train motor with ID=" + strSlaveDeviceID + 
          " voltage compensation to 12 Volts", "Error");
      }
      // Sets the idle mode setting for the conveyor SPARK MAX as brake.
      if(slave.setIdleMode(IdleMode.kBrake) != CANError.kOk) {
        SmartDashboard.putString("Failed to set slave drive train motor with ID=" + strSlaveDeviceID + 
          " idle mode to Brake", "Error");
      }
      // Sets the ramp rate for open loop control modes. 
      if(slave.setOpenLoopRampRate(.2) != CANError.kOk) {
        SmartDashboard.putString("Failed to set slave drive train motor with ID=" + strSlaveDeviceID + 
          " ramp rate for open loop control modes", "Error");
      }
      // Sets the current limit in Amps for master drive train motors at 40A
      if(slave.setSmartCurrentLimit(40) != CANError.kOk) {
        SmartDashboard.putString("Failed to set the current limit in Amps for slave drive train motor with ID=" + strSlaveDeviceID + 
          " at 40A", "Error");
      }
      // Causes slave controller's output to mirror the master controller
      if(slave.follow(master) != CANError.kOk) {
        SmartDashboard.putString("Failed to set slave controller's with ID=" + strSlaveDeviceID + 
          " output to mirror the master controller", "Error");
      }
    }
  }

  /** 
   * Initialize Integrated PID left and right controllers
   */
  public void setTurnPID(){
    // Set the Proportional Gain constant of the PIDF controller on the SPARK MAX. 
    if(l_pidController.setP(.07) != CANError.kOk) {
      SmartDashboard.putString("Failed to set the Proportional Gain constant of the left PIDF controller on the SPARK MAX", "Error");
    }
    // Set the Integral Gain constant of the PIDF controller on the SPARK MAX.
    if(l_pidController.setI(0) != CANError.kOk) {
      SmartDashboard.putString("Failed to set the Integral Gain constant of the left PIDF controller on the SPARK MAX.", "Error");
    }
    // Set the Derivative Gain constant of the PIDF controller on the SPARK MAX. 
    if(l_pidController.setD(0) != CANError.kOk) {
      SmartDashboard.putString("Failed to set the Derivative Gain constant of the left PIDF controller on the SPARK MAX.", "Error");
    }
    // Set the IZone range of the PIDF controller on the SPARK MAX.
    if(l_pidController.setIZone(0) != CANError.kOk) {
      SmartDashboard.putString("Failed to set the IZone range of the left PIDF controller on the SPARK MAX.", "Error");
    }
    // Set the Feed-froward Gain constant of the PIDF controller on the SPARK MAX. 
    if(l_pidController.setFF(0) != CANError.kOk) {
      SmartDashboard.putString("Failed to set the Feed-froward Gain constant of the left PIDF controller on the SPARK MAX.", "Error");
    }
    // Set the Proportional Gain constant of the PIDF controller on the SPARK MAX.
    if(r_pidController.setP(.07) != CANError.kOk) {
      SmartDashboard.putString("Failed to set the Proportional Gain constant of the right PIDF controller on the SPARK MAX", "Error");
    }
    // Set the Integral Gain constant of the PIDF controller on the SPARK MAX.
    if(r_pidController.setI(0) != CANError.kOk) {
      SmartDashboard.putString("Failed to set the Integral Gain constant of the right PIDF controller on the SPARK MAX.", "Error");
    }
    // Set the Derivative Gain constant of the PIDF controller on the SPARK MAX.
    if(r_pidController.setD(0) != CANError.kOk) {
      SmartDashboard.putString("Failed to set the Derivative Gain constant of the right PIDF controller on the SPARK MAX.", "Error");
    }
    // Set the IZone range of the PIDF controller on the SPARK MAX.
    if(r_pidController.setIZone(0) != CANError.kOk) {
      SmartDashboard.putString("Failed to set the IZone range of the right PIDF controller on the SPARK MAX.", "Error");
    }
    // Set the Feed-froward Gain constant of the PIDF controller on the SPARK MAX.
    if(r_pidController.setFF(0) != CANError.kOk) {
      SmartDashboard.putString("Failed to set the Feed-froward Gain constant of the right PIDF controller on the SPARK MAX.", "Error");
    }
  }

  /** 
   * Set Integrated PID for left and right controllers
   */
  public void setNormalPID(){
    // Set the Proportional Gain constant of the PIDF controller on the SPARK MAX. 
    if(l_pidController.setP(kP) != CANError.kOk) {
      SmartDashboard.putString("Failed to set the Proportional Gain constant of the left PIDF controller on the SPARK MAX", "Error");
    }
    // Set the Integral Gain constant of the PIDF controller on the SPARK MAX.
    if(l_pidController.setI(kI) != CANError.kOk) {
      SmartDashboard.putString("Failed to set the Integral Gain constant of the left PIDF controller on the SPARK MAX.", "Error");
    }
    // Set the Derivative Gain constant of the PIDF controller on the SPARK MAX. 
    if(l_pidController.setD(kD) != CANError.kOk) {
      SmartDashboard.putString("Failed to set the Derivative Gain constant of the left PIDF controller on the SPARK MAX.", "Error");
    }
    // Set the IZone range of the PIDF controller on the SPARK MAX.
    if(l_pidController.setIZone(kIz) != CANError.kOk) {
      SmartDashboard.putString("Failed to set the IZone range of the left PIDF controller on the SPARK MAX.", "Error");
    }
    // Set the Feed-froward Gain constant of the PIDF controller on the SPARK MAX. 
    if(l_pidController.setFF(kFF) != CANError.kOk) {
      SmartDashboard.putString("Failed to set the Feed-froward Gain constant of the left PIDF controller on the SPARK MAX.", "Error");
    }
    // Set the Proportional Gain constant of the PIDF controller on the SPARK MAX.
    if(r_pidController.setP(kP) != CANError.kOk) {
      SmartDashboard.putString("Failed to set the Proportional Gain constant of the right PIDF controller on the SPARK MAX", "Error");
    }
    // Set the Integral Gain constant of the PIDF controller on the SPARK MAX.
    if(r_pidController.setI(kI) != CANError.kOk) {
      SmartDashboard.putString("Failed to set the Integral Gain constant of the right PIDF controller on the SPARK MAX.", "Error");
    }
    // Set the Derivative Gain constant of the PIDF controller on the SPARK MAX.
    if(r_pidController.setD(kD) != CANError.kOk) {
      SmartDashboard.putString("Failed to set the Derivative Gain constant of the right PIDF controller on the SPARK MAX.", "Error");
    }
    // Set the IZone range of the PIDF controller on the SPARK MAX.
    if(r_pidController.setIZone(kIz) != CANError.kOk) {
      SmartDashboard.putString("Failed to set the IZone range of the right PIDF controller on the SPARK MAX.", "Error");
    }
    // Set the Feed-froward Gain constant of the PIDF controller on the SPARK MAX.
    if(r_pidController.setFF(kFF) != CANError.kOk) {
      SmartDashboard.putString("Failed to set the Feed-froward Gain constant of the right PIDF controller on the SPARK MAX.", "Error");
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