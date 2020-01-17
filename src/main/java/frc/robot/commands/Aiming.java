/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.*;

import java.util.logging.Logger;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.*;

/**
 * An example command that uses an example subsystem.
 */
public class Aiming extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private static final Logger LOGGER = Logger.getLogger(DriveCommand.class.getName());
  private Joystick joystick = new Joystick(OI.joystick);
  double tx = SmartDashboard.getNumber("LimelightX", 0);
  double minSteerAdjust = .2;
  double steering_adjust = 0.0;
  double headingCommand = 0;
  double p = .013;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Aiming() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.mDriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.m_robotContainer.light.setValue(3);
    tx = SmartDashboard.getNumber("LimelightX", 0);
    if(tx>1.0){
        steering_adjust = p * tx +minSteerAdjust;
    }
    else if(tx<-1.0){
        steering_adjust = p * tx -minSteerAdjust;
    }
     headingCommand = headingCommand + steering_adjust;
     
    if(tx != 0)
    RobotContainer.mDriveSubsystem.arcadeDrive(joystick.getRawAxis(1),headingCommand);
    else
    RobotContainer.mDriveSubsystem.arcadeDrive(joystick.getRawAxis(1),joystick.getRawAxis(2));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.light.setValue(1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    //return ((tx<1.0  && tx> -1.0) && tx != 0);
  }
}
