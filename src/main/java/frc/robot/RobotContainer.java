/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.logging.Logger;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Aiming;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.RunTrajectory;
import frc.robot.commands.toggleOff;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.trajectory.*;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer{
  // The robot's subsystems and commands are defined here...
  public static OI mOI = new OI();
  public static DriveSubsystem mDriveSubsystem = new DriveSubsystem();
  public static DriveCommand mDriveCommand = new DriveCommand();
  private static final Logger LOGGER = Logger.getLogger(Robot.class.getName());
  private static Trajectory trajectory1;
  private static Trajectory trajectory2;
  public static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  public static NetworkTableEntry tx = table.getEntry("tx");
  public static NetworkTableEntry ty = table.getEntry("ty");
  public static NetworkTableEntry ta = table.getEntry("ta");
  public static NetworkTableEntry light = table.getEntry("ledMode");


  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    try{
    trajectory1 = TrajectoryUtil.fromPathweaverJson(Paths.get("/paths/Rook6f.json"));
    trajectory2 = TrajectoryUtil.fromPathweaverJson(Paths.get("/paths/Knight6f5l.json"));
    }catch(Exception e){
      e.printStackTrace();
    }
    
     configureButtonBindings();
     mDriveSubsystem.setDefaultCommand(mDriveCommand);
   }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    mOI.buttonOne.whenPressed(new RunTrajectory(trajectory1));
    mOI.buttonThree.whenPressed(new RunTrajectory(trajectory2));
    mOI.buttonTwo.whenPressed(new toggleOff());
    mOI.buttonFive.whileHeld(new Aiming(), false);
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // A RunTrajectory command will run in autonomous
    RunTrajectory runTrajectory = new RunTrajectory(trajectory2);
    return runTrajectory.getCommand();
  }
}
