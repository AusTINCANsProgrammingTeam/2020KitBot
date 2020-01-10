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

import java.util.logging.Logger;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class DriveSetSpeed extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private static final Logger LOGGER = Logger.getLogger(DriveCommand.class.getName());
  private Joystick joystick = new Joystick(OI.joystick);
  private double speedForward;
  private double speedHeading;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveSetSpeed(double speedForward, double speedHeading) {
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
      RobotContainer.mDriveSubsystem.arcadeDrive(speedForward, speedHeading);
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
