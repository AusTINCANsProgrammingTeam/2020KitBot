/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.*;

import java.util.logging.Logger;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class TurnAimShoot extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private static final Logger LOGGER = Logger.getLogger(DriveCommand.class.getName());
  double tx = SmartDashboard.getNumber("LimelightX", 0);
  double ty = SmartDashboard.getNumber("LimelightY", 0);
  double ta = SmartDashboard.getNumber("LimelightArea", 0);
  double d = 0;
  
  double minSteerAdjust = .25;
  double steeringAdjust = 0.0;
  double headingCommand = 0;
  double p = .013;
  double targetValue = -3;
  boolean seen = false;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TurnAimShoot() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.mDriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.light.setValue(Constants.LL_LIGHT_ON);
  }

  // Called every time the scheduler runs while the command is scheduled.
  // tx is the degrees the limelight detects we are off by the target 
  // adjust p until it works
  @Override
  public void execute() {
    tx = SmartDashboard.getNumber("LimelightX", 0);
    ty = SmartDashboard.getNumber("LimelightY", 0);
    ta = SmartDashboard.getNumber("LimelightArea", 0);
    if(tx>(targetValue +1)){
      steeringAdjust = p * (tx-targetValue) +minSteerAdjust;
   }
   else if(tx<(targetValue -1)){
      steeringAdjust = p * (tx-targetValue) -minSteerAdjust;
   }

   if (tx <= (targetValue +1) && tx >= (targetValue -1) && ShooterSubsystem.hoodedShooter.get() == Value.kForward && ta != 0 ) {
    RobotContainer.mShooterSubsystem.toggleHood();
    seen = true;
  }
  if (ta != 0)
  RobotContainer.mDriveSubsystem.arcadeDrive(0, steeringAdjust);
      steeringAdjust = 0;
      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.light.setValue(Constants.LL_LIGHT_OFF);
    seen = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (tx < targetValue +1 && tx > targetValue -1 && seen == true);
  }
}
