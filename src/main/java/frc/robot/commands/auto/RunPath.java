/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import java.util.Iterator;
import java.util.List;
import java.util.logging.Logger;


/**
 * An example command that uses an example subsystem.
 */
public class RunPath extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private List rightPath;
  private List leftPath;
  int i = 0;
  private Iterator<Double> m_leftIterator;
  private Iterator<Double> m_rightIterator;
  private static final Logger LOGGER = Logger.getLogger(Robot.class.getName());


  public RunPath(List leftPath, List rightPath) {
    this.rightPath = rightPath;
    this.leftPath = leftPath;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.mDriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_leftIterator = leftPath.iterator();
    m_rightIterator = rightPath.iterator();
    i = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    if(i< leftPath.size()){
      RobotContainer.mDriveSubsystem.setLeftPidVelocitySetpoint(-1*RobotContainer.mDriveSubsystem.fpsToRPM(m_leftIterator.next()));
      RobotContainer.mDriveSubsystem.setRightPidVelocitySetpoint(RobotContainer.mDriveSubsystem.fpsToRPM(m_rightIterator.next()));
      SmartDashboard.putNumber("Left Commanded Velocity", -1*RobotContainer.mDriveSubsystem.fpsToRPM(Double.valueOf(leftPath.get(i).toString())));    
      SmartDashboard.putNumber("Right Commanded Velocity", RobotContainer.mDriveSubsystem.fpsToRPM(Double.valueOf(rightPath.get(i).toString())));
      i++;
    }
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_rightIterator.remove();
    m_leftIterator.remove();
    RobotContainer.mDriveSubsystem.setRightSetpoint(0);
    RobotContainer.mDriveSubsystem.setLeftPidVelocitySetpoint(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return i >= leftPath.size();  
    //return timer.get() >= timeToRun;
  }
}
