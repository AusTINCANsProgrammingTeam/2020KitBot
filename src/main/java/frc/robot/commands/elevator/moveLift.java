/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

/**
 * An example command that uses an example subsystem.
 */
public class moveLift extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  double axis;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public moveLift() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.mElevatorSubystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      RobotContainer.mElevatorSubystem.moveLift(RobotContainer.OperatorController.getRawAxis(1));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return false;
  }
}
