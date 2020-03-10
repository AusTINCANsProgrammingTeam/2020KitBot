/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

import java.util.logging.Logger;

/**
 * An example command that uses an example subsystem.
 */
public class autoHopperVortex extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  int i;
  double timeRun;
  private static final Logger LOGGER = Logger.getLogger(autoHopperVortex.class.getName());

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public autoHopperVortex( double timeRun) {
    this.timeRun = timeRun;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      i=0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      i++;
      RobotContainer.mHopperSubsystem.runIntake(0,.25);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      RobotContainer.mHopperSubsystem.stopIntake();
      //LOGGER.warning("AUTO HOPPER DONE");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((i*.02) >=timeRun);
  }
}
