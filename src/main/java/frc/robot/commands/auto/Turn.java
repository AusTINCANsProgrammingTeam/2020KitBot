/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class Turn extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    int i =0;
    public Turn(int i) {
        this.i = i;
        addRequirements(RobotContainer.mDriveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        RobotContainer.mDriveSubsystem.setTurnPID();
        DriveSubsystem.l_encoder.setPosition(0);
        DriveSubsystem.r_encoder.setPosition(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      RobotContainer.mDriveSubsystem.setLeftPidPositionSetpoint(i*24.5);
      RobotContainer.mDriveSubsystem.setRightPidPositionSetpoint(i*24.5);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.mDriveSubsystem.setNormalPID();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(i>0)
    return RobotContainer.mDriveSubsystem.l_encoder.getPosition() >= 24.2;
    else
    return RobotContainer.mDriveSubsystem.l_encoder.getPosition() <= -24.2;

  }
}
