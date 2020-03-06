/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ExampleSubsystem;

import java.util.logging.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class autoStoreValue extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private static final Logger LOGGER = Logger.getLogger(autoStoreValue.class.getName());

  private double targetValue = 0;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public autoStoreValue() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.light.setValue(Constants.LL_LIGHT_ON);
    SmartDashboard.putNumber("Data Offset",  SmartDashboard.getNumber("LimelightX", 0));
    targetValue = SmartDashboard.getNumber("Data Offset", 0);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.light.setValue(Constants.LL_LIGHT_OFF);
    //LOGGER.warning("STORE VALUE DONE");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (targetValue != 0) ;
  }
}
