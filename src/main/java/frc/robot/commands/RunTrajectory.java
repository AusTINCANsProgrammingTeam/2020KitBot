package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.DriveConstants;
import frc.robot.AutoConstants;

import java.util.List;
import java.util.function.Supplier;
import java.util.logging.Logger;

import com.revrobotics.CANPIDController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
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

  private DriveSubsystem driveSubsystem;

  public RunTrajectory(Trajectory trajectory){
    // Run path following command, then stop at the end.

    driveSubsystem = RobotContainer.mDriveSubsystem;

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
      driveSubsystem::getPose,
      new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
      new SimpleMotorFeedforward (
        DriveConstants.ksVolts,
        DriveConstants.kvVoltSecondsPerMeter,
        DriveConstants.kaVoltSecondsSquaredPerMeter
      ),
      DriveConstants.kDriveKinematics,
      driveSubsystem::getWheelSpeeds,
      new PIDController(DriveConstants.kPDriveVel, 0, 0),
      new PIDController(DriveConstants.kPDriveVel, 0, 0),
      // RamseteCommand passes volts to the callback
      RobotContainer.mDriveSubsystem::tankDriveVolts, 
      RobotContainer.mDriveSubsystem
    );
 
      
    }
    
    public Command getCommand() {
      return ramseteCommand.andThen(() -> RobotContainer.mDriveSubsystem.tankDriveVolts(0, 0));
    }
}
