/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.logging.Logger;

/**
 * An example command that uses an example subsystem.
 */
public class Aiming extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private static final Logger LOGGER = Logger.getLogger(DriveCommand.class.getName());
  double tx = SmartDashboard.getNumber("LimelightX", 0);
  double ty = SmartDashboard.getNumber("LimelightY", 0);
  double ta = SmartDashboard.getNumber("LimelightArea", 0);
  double d = 0;

  double minSteerAdjust = .25;
  double steeringAdjust = 0.0;
  double headingCommand = 0;
  double p = .013;

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
    d = 64/Math.tan(Math.toRadians(ty+16));
    SmartDashboard.putNumber("Distance", d);
    double targetValue = -3;
    if(tx>(targetValue +1)){
      steeringAdjust = p * (tx-targetValue) +minSteerAdjust;
   }
   else if(tx<(targetValue -1)){
      steeringAdjust = p * (tx-targetValue) -minSteerAdjust;
   }

    if (tx <= (targetValue +1) && tx >= (targetValue -1) && ShooterSubsystem.hoodedShooter.get() == Value.kForward && ta != 0 ) {
      RobotContainer.mShooterSubsystem.toggleHood();
    }

    if (tx != 0)
      RobotContainer.mDriveSubsystem.arcadeDrive(RobotContainer.DriverController.getRawAxis(1), steeringAdjust);
    else
      RobotContainer.mDriveSubsystem.arcadeDrive(RobotContainer.DriverController.getRawAxis(1),
          RobotContainer.DriverController.getRawAxis(2));
    steeringAdjust = 0;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.light.setValue(Constants.LL_LIGHT_OFF);
    if (ShooterSubsystem.hoodedShooter.get() == Value.kReverse){
      RobotContainer.mShooterSubsystem.toggleHood();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
