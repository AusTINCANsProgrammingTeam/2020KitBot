/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import java.util.logging.*;


public class ElevatorSubsystem extends SubsystemBase {
    private CANSparkMax motor1 = new CANSparkMax(5, MotorType.kBrushless);;
    private CANSparkMax motor2 = new CANSparkMax(10, MotorType.kBrushless);;
    private CANPIDController pidController;
    private CANEncoder encoder;
    public double pos = 0;
    public double kP = 1e-7, kI, kD, kIz, kFF,
     kMaxOutput = 1, kMinOutput = -1;
    private static final Logger LOGGER = Logger.getLogger(DriveSubsystem.class.getName());

  public ElevatorSubsystem() {
    motor2.follow(motor1, true);

    pidController = motor1.getPIDController();

    encoder = motor1.getEncoder();

    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF);
    pidController.setOutputRange(kMinOutput, kMaxOutput);
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("FF Value", kFF);
  }
 public void liftUp(double position){
     pidController.setReference(position, ControlType.kPosition);
}

public void incrementLift(){

    pidController.setReference(pos++, ControlType.kPosition);
}

public void incrementLiftDown(){

    pidController.setReference(pos--, ControlType.kPosition);
}



public double getPosition(){
    return encoder.getPosition();
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}