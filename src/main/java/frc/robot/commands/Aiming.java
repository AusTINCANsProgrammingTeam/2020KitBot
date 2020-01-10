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
    double tx = SmartDashboard.getNumber("LimelightX", 0);
    double min_command = .1;
    double steering_adjust = 0.0;
    if(tx>1.0){
        steering_adjust = .03 * tx;
    }
    else if(tx<1.0){
        steering_adjust = .03 * tx;
    }
    if(steering_adjust > min_command)
    RobotContainer.mDriveSubsystem.arcadeDrive(0.0,steering_adjust);
    else
    RobotContainer.mDriveSubsystem.arcadeDrive(0.0,min_command);
    LOGGER.warning("test");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.mDriveCommand.schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
