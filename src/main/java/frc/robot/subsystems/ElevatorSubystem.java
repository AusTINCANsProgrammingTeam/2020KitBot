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
import frc.robot.RobotContainer;
import frc.robot.commands.elevator.moveLift;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import java.util.logging.*;


public class ElevatorSubystem extends SubsystemBase {
    private static CANSparkMax motor1 = new CANSparkMax(Constants.Elevator1,MotorType.kBrushless);
    private static CANSparkMax motor2 = new CANSparkMax(Constants.Elevator2, MotorType.kBrushless);
    private CANPIDController pidController;
    private CANEncoder encoder;
    private boolean readyUse = false;
    private double position = 0;
    public double kP= .01, kI=0, kD=0, kIz, kFF,
     kMaxOutput = 1, kMinOutput = -1;
    private static final Logger LOGGER = Logger.getLogger(DriveSubsystem.class.getName());

  public ElevatorSubystem() {

    motor1.restoreFactoryDefaults();
    motor2.restoreFactoryDefaults();
    motor2.follow(motor1, true);
    
    pidController = motor1.getPIDController();
    motor1.setSmartCurrentLimit(40);
    motor2.setSmartCurrentLimit(40);

    encoder = motor1.getEncoder();
    encoder.setPosition(0);

    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF);
    pidController.setOutputRange(kMinOutput, kMaxOutput);
  }
public void moveLift(double joystickInput){
  if(readyUse == true){
    if((joystickInput < 0 && position > -570) || (joystickInput > 0 && position < 0))
      position = position+(10*joystickInput);
    if(position >=-570 && position <= 0)
      pidController.setReference(position, ControlType.kPosition);
    SmartDashboard.putNumber("commanded position", position);
  }
    
}

public double getPosition(){
    return encoder.getPosition();
}
public void armElevator(){
  readyUse = true;
}

public boolean getReadyUse(){
  return readyUse;
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

