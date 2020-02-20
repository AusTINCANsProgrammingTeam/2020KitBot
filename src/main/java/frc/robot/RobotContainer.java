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
import frc.robot.commands.Aiming;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.RunPath;
import frc.robot.commands.RunPathBack;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TurnAimShoot;
import frc.robot.commands.autoDeCorrect;
import frc.robot.commands.autoStoreValue;
import frc.robot.commands.hopperIn;
import frc.robot.commands.liftUp;
import frc.robot.commands.runIntakeIn;
import frc.robot.commands.runIntakeOut;
import frc.robot.commands.toggleIntake;
import frc.robot.commands.toggleOff;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubystem;
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
  public static Joystick controller2 = new Joystick(1);
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


  public final static int joystick = 0;
  public static DriveSubsystem mDriveSubsystem = new DriveSubsystem();
  public static DriveCommand mDriveCommand = new DriveCommand();
  public static ShooterSubsystem mShooterSubsystem = new ShooterSubsystem();
  public static IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem();
  public static ElevatorSubystem mElevatorSubystem = new ElevatorSubystem();
  public static HopperSubsystem mHopperSubsystem = new HopperSubsystem();

  private static final Logger LOGGER = Logger.getLogger(Robot.class.getName());
  private static Path path1 = new Path("trenchAndAim");
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
     }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
    buttonOne.whenPressed(new autoStoreValue());
    buttonFive.whileHeld(new Aiming(), false);
    buttonSix.whenPressed(new SequentialCommandGroup(new RunPath(leftArray1, rightArray1), 
    new autoStoreValue(), new TurnAimShoot(), new autoDeCorrect(), new RunPath(leftArray2, rightArray2), new RunPathBack(leftArray2, rightArray2)));
    buttonFour.whileHeld(new ShootCommand(), false);
    buttonSeven.whenPressed(new toggleIntake());
    buttonTwo.whileHeld(new ParallelCommandGroup(new runIntakeIn(), new hopperIn()));
    buttonEight.whenPressed(new liftUp());

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
