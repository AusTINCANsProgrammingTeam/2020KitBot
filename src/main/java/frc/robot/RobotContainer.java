/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.logging.Logger;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.Path;
import frc.robot.commands.auto.RunPath;
import frc.robot.commands.drive.Aiming;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.elevator.ArmElevator;
import frc.robot.commands.elevator.moveLift;
import frc.robot.commands.hopper.hopperIn;
import frc.robot.commands.hopper.hopperOut;
import frc.robot.commands.intake.runIntakeIn;
import frc.robot.commands.intake.runIntakeOut;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.commands.shooter.conveyorIn;
import frc.robot.commands.shooter.conveyorOut;
import frc.robot.commands.shooter.conveyorShooter;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static Joystick DriverController = new Joystick(0);
  public static Joystick OperatorController = new Joystick(1);
  public JoystickButton buttonOneDrive = new JoystickButton(DriverController, 1);  
  public JoystickButton buttonTwoDrive = new JoystickButton(DriverController, 2);  
  public JoystickButton buttonThreeDrive = new JoystickButton(DriverController, 3);  
  public JoystickButton buttonFourDrive = new JoystickButton(DriverController, 4);
  public JoystickButton buttonFiveDrive = new JoystickButton(DriverController, 5);
  public JoystickButton buttonSixDrive = new JoystickButton(DriverController, 6);
  public JoystickButton buttonSevenDrive = new JoystickButton(DriverController, 7);
  public JoystickButton buttonEightDrive = new JoystickButton(DriverController, 8);
  public JoystickButton buttonNineDrive = new JoystickButton(DriverController, 9);

  public JoystickButton buttonOneOp = new JoystickButton(OperatorController, 1);  
  public JoystickButton buttonTwoOp = new JoystickButton(OperatorController, 2);  
  public JoystickButton buttonThreeOp = new JoystickButton(OperatorController, 3);  
  public JoystickButton buttonFourOp = new JoystickButton(OperatorController, 4);
  public JoystickButton buttonFiveOp = new JoystickButton(OperatorController, 5);
  public JoystickButton buttonSixOp = new JoystickButton(OperatorController, 6);
  public JoystickButton buttonSevenOp = new JoystickButton(OperatorController, 7);
  public JoystickButton buttonEightOp = new JoystickButton(OperatorController, 8);
  public JoystickButton buttonNineOp = new JoystickButton(OperatorController, 9);
  public JoystickButton buttonTenOp = new JoystickButton(OperatorController, 10);


  public static DriveSubsystem mDriveSubsystem = new DriveSubsystem();
  public static DriveCommand mDriveCommand = new DriveCommand();
  public static ShooterSubsystem mShooterSubsystem = new ShooterSubsystem();
  public static IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem();
  public static ElevatorSubystem mElevatorSubystem = new ElevatorSubystem();
  public static HopperSubsystem mHopperSubsystem = new HopperSubsystem();
  public static ConveyorSubsystem mConveyorSubsystem = new ConveyorSubsystem();

  private static final Logger LOGGER = Logger.getLogger(Robot.class.getName());
  private static Path path1 = new Path("10S-5L");
  private static Path path2 = new Path("DriveTrench");
  private static ArrayList<Double> leftArray1;
  private static ArrayList<Double> rightArray1;
  private static ArrayList<Double> leftArray2;
  private static ArrayList<Double> rightArray2;
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
      rightArray1 = path1.returnRightList();
      leftArray1 = path1.returnLeftList();
      rightArray2 = path2.returnRightList();
      leftArray2 = path2.returnLeftList();
    }
      catch(IOException e){
        LOGGER.warning("real really test");
      }
     configureButtonBindings();
     mDriveSubsystem.setDefaultCommand(mDriveCommand);
     mElevatorSubystem.setDefaultCommand(new moveLift());

     }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //driver configuration
    buttonFiveDrive.whileHeld(new ParallelCommandGroup(new ShootCommand(), new hopperIn(), new conveyorShooter()));
    buttonSixDrive.whileHeld(new Aiming(), false);
    //operator configuration
    buttonSixOp.whileHeld(new ParallelCommandGroup(new runIntakeOut(), new hopperOut()));
    buttonEightOp.whileHeld(new ParallelCommandGroup(new runIntakeIn(), new hopperIn()));
    buttonSevenOp.whileHeld(new ParallelCommandGroup(new conveyorOut(), new hopperOut()));
    buttonFiveOp.whileHeld(new ParallelCommandGroup(new hopperIn(), new conveyorIn()));
    buttonTenOp.whenPressed(new ArmElevator());

  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new RunPath(leftArray2, rightArray2);
  }
}
