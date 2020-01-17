/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

import java.util.List;
import java.util.logging.Logger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * An example command that uses an example subsystem.
 */
public class RunPath extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private List rightPath;
  private List leftPath;
  private double timeToRun;
  private Timer timer;
  int i = 0;
  private static final Logger LOGGER = Logger.getLogger(Robot.class.getName());


  public RunPath(List leftPath, List rightPath) {
    this.rightPath = rightPath;
    this.leftPath = leftPath;
    timeToRun = .020 * (leftPath.size()-1);
    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.mDriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    i = 0;
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.mDriveSubsystem.setLeftPidVelocitySetpoint(-1*RobotContainer.mDriveSubsystem.fpsToRPM(Double.valueOf(leftPath.get(i).toString())));
    RobotContainer.mDriveSubsystem.setRightPidVelocitySetpoint(RobotContainer.mDriveSubsystem.fpsToRPM(Double.valueOf(rightPath.get(i).toString())));
    SmartDashboard.putNumber("Left Commanded Velocity", RobotContainer.mDriveSubsystem.fpsToRPM(Double.valueOf(leftPath.get(i).toString())));    
    SmartDashboard.putNumber("Right Commanded Velocity", RobotContainer.mDriveSubsystem.fpsToRPM(Double.valueOf(rightPath.get(i).toString())));

    i++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.mDriveSubsystem.setLeftPidVelocitySetpoint(0);
    RobotContainer.mDriveSubsystem.setRightPidVelocitySetpoint(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return timer.get() >= timeToRun;
  }
}
