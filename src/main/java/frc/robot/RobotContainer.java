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

import com.analog.adis16448.frc.ADIS16448_IMU;
import com.analog.adis16448.frc.ADIS16448_IMU.IMUAxis;

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
import frc.robot.commands.drive.Aiming;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.elevator.elevatorDown;
import frc.robot.commands.elevator.elevatorUp;
import frc.robot.commands.intake.hopperIn;
import frc.robot.commands.intake.runIntakeIn;
import frc.robot.commands.intake.toggleIntake;
import frc.robot.commands.auto.RunPath;
import frc.robot.commands.auto.RunPathBack;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.commands.auto.Turn;

import frc.robot.commands.auto.TurnAimShoot;
import frc.robot.commands.auto.autoDeCorrect;
import frc.robot.commands.auto.autoShoot;
import frc.robot.commands.auto.autoStoreValue;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Path;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static Joystick controller1 = new Joystick(0);
  public static Joystick controller2 = new Joystick(2);
  public JoystickButton buttonOne = new JoystickButton(controller1, 1);  
  public JoystickButton buttonTwo = new JoystickButton(controller1, 2);  
  public JoystickButton buttonThree = new JoystickButton(controller1, 3);  
  public JoystickButton buttonFour = new JoystickButton(controller1, 4);
  public JoystickButton buttonFive = new JoystickButton(controller1, 5);
  public JoystickButton buttonSix = new JoystickButton(controller1, 6);
  public JoystickButton buttonSeven = new JoystickButton(controller1, 7);
  public JoystickButton buttonEight = new JoystickButton(controller1, 8);
  public JoystickButton buttonNine = new JoystickButton(controller1, 9);

  public JoystickButton buttonOneOp = new JoystickButton(controller2, 1);  
  public JoystickButton buttonTwoOp = new JoystickButton(controller2, 2);  
  public JoystickButton buttonThreeOp = new JoystickButton(controller2, 3);  
  public JoystickButton buttonFourOp = new JoystickButton(controller2, 4);
  public JoystickButton buttonFiveOp = new JoystickButton(controller2, 5);
  public JoystickButton buttonSixOp = new JoystickButton(controller2, 6);
  public JoystickButton buttonSevenOp = new JoystickButton(controller2, 7);
  public JoystickButton buttonEightOp = new JoystickButton(controller2, 8);
  public JoystickButton buttonNineOp = new JoystickButton(controller2, 9);

  public static ElevatorSubsystem mElevatorSubsystem = new ElevatorSubsystem();
  public static DriveSubsystem mDriveSubsystem = new DriveSubsystem();
  public static HopperSubsystem mHopperSubsystem = new HopperSubsystem();
  public static IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem();
  public static DriveCommand mDriveCommand = new DriveCommand();
  public static ShooterSubsystem mShooterSubsystem = new ShooterSubsystem();
  private static final Logger LOGGER = Logger.getLogger(Robot.class.getName());
  private static Path path1 = new Path("fast6Rook");
  private static Path path2 = new Path("DriveTrench");
  private static Path turnPath = new Path("turn180");
  protected static ArrayList<Double> leftArray1;
  protected static ArrayList<Double>  rightArray1;
  protected static ArrayList<Double>  leftArray2;
  protected static ArrayList<Double>  rightArray2;
  protected static ArrayList<Double>  leftTurnArray;
  protected static ArrayList<Double>  rightTurnArray;
  public static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  public static NetworkTableEntry tx = table.getEntry("tx");
  public static NetworkTableEntry ty = table.getEntry("ty");
  public static NetworkTableEntry ta = table.getEntry("ta");
  public static NetworkTableEntry light = table.getEntry("ledMode");
  private static String autoName = "leftRook";



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
      leftTurnArray = turnPath.returnLeftList();
      rightTurnArray = turnPath.returnRightList();
      
    }
      catch(IOException e){
        LOGGER.warning("real really test");
      }
    AutoGroups.startAutoGroups();
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
    buttonOne.whenPressed(new Turn(leftTurnArray, rightTurnArray));
    buttonFive.whileHeld(new Aiming(), false);
    buttonFour.whileHeld(new ShootCommand(), false);
    buttonSeven.whenPressed(new toggleIntake());
    buttonTwo.whileHeld(new ParallelCommandGroup(new runIntakeIn(), new hopperIn()));
    // buttonEight.whileHeld(new elevatorUp(), false);
    // buttonNine.whileHeld(new elevatorDown(), false);
    // buttonThree.whenPressed(new RunPathBack(leftArray1, rightArray1));
    buttonThree.whenPressed(new SequentialCommandGroup(new RunPathBack(leftArray1, rightArray2), new Turn(leftTurnArray, rightTurnArray), 
     new ParallelCommandGroup(new RunPath(leftArray1, rightArray1), new runIntakeIn(), new hopperIn())));
    //buttonSix.whenPressed(AutoGroups.getAutoGroup(autoName));

    //mOI.buttonThree.whenPressed(new SequentialCommandGroup(new autoStoreValue(), new TurnAimShoot(), new autoDeCorrect()));
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
