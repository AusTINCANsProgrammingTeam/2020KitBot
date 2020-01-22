/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.RobotContainer;

import java.util.List;
import java.util.logging.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.*;
import edu.wpi.first.wpilibj.trajectory.constraint.*;




/**
 * An example command that uses an example subsystem.
 */
public class RunTrajectory extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private Trajectory trajectory;
  private static final Logger LOGGER = Logger.getLogger(Robot.class.getName());

  private DifferentialDriveVoltageConstraint autoVoltageConstraint;
  private TrajectoryConfig config;
  private RamseteCommand ramseteCommand;

  public RunTrajectory(Trajectory trajectory){
    // Run path following command, then stop at the end.
    this.trajectory = trajectory;
      
    this.autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(
        DriveConstants.ksVolts,
        DriveConstants.kvVoltSecondsPerMeter,
        DriveConstants.kaVoltSecondsSquaredPerMeter
      ),
      DriveConstants.kDriveKinematics,
      10
    );

    this.config = new TrajectoryConfig (
      AutoConstants.kMaxSpeedMetersPerSecond,
      AutoConstants.kMaxAccelerationMetersPerSecondSquared
    )
      // Add kinematics to ensure max speed is actually obeyed
    .setKinematics(DriveConstants.kDriveKinematics)
      // Apply the voltage constraint
    .addConstraint(autoVoltageConstraint);

    this.ramseteCommand = new RamseteCommand(
      trajectory,
      RobotContainer.mDriveSubsystem::getPose,
      new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
      new SimpleMotorFeedforward (
        DriveConstants.ksVolts,
        DriveConstants.kvVoltSecondsPerMeter,
        DriveConstants.kaVoltSecondsSquaredPerMeter
      ),
      DriveConstants.kDriveKinematics,
      RobotContainer.mDriveSubsystem::getWheelSpeeds,
      new PIDController(DriveConstants.kPDriveVel, 0, 0),
      new PIDController(DriveConstants.kPDriveVel, 0, 0),
      // RamseteCommand passes volts to the callback
      RobotContainer.mDriveSubsystem::tankDriveVolts, RobotContainer.mDriveSubsystem
    );
 
      
    }
    
    public Command getCommand() {
      return ramseteCommand.andThen(() -> RobotContainer.mDriveSubsystem.tankDriveVolts(0, 0));
    }
}

