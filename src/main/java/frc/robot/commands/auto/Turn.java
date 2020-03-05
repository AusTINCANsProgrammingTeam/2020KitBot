/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import frc.robot.RobotContainer;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class Turn extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Turn() {
      addRequirements(RobotContainer.mDriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      RobotContainer.mDriveSubsystem.l_encoder.setPosition(0);
      RobotContainer.mDriveSubsystem.r_encoder.setPosition(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      RobotContainer.mDriveSubsystem.setLeftPidPositionSetpoint(25);
      RobotContainer.mDriveSubsystem.setRightPidPositionSetpoint(-25);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.mDriveSubsystem.l_encoder.getPosition() >= 25;
  }
}
