/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class autoShoot extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private double i;
  private double runTime;
  double tx = SmartDashboard.getNumber("LimelightX", 0);
  double ty = SmartDashboard.getNumber("LimelightY", 0);
 
  double minSteerAdjust = .2;
  double steeringAdjust = 0.0;
  
  double headingCommand = 0;
  double p = .013;
  boolean seen = false;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  double maxRPM = 5700;
  public autoShoot(double runTime) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.mShooterSubsystem);
    i = 0;
    this.runTime = runTime;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    i = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      
    RobotContainer.light.setValue(Constants.LL_LIGHT_ON);
    tx = SmartDashboard.getNumber("LimelightX", 0);
    ty = SmartDashboard.getNumber("LimelightY", 0);
    if(tx>.5){
        steeringAdjust = p * tx +minSteerAdjust;
        seen = true;
    }
    else if(tx<-.5){
        steeringAdjust = p * tx -minSteerAdjust;
        seen = true;
    }
     
    if(tx != 0)
      RobotContainer.mDriveSubsystem.arcadeDrive(0,steeringAdjust);

      steeringAdjust = 0;
  
      SmartDashboard.putNumber("Shooter RPM", RobotContainer.mShooterSubsystem.shooterVelocity());
      RobotContainer.mShooterSubsystem.setVelocitySetpoint(.60*maxRPM);
      
      i++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      RobotContainer.mShooterSubsystem.setVelocitySetpoint(0);
      RobotContainer.light.setValue(Constants.LL_LIGHT_OFF);
      seen = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return i > (runTime/.02) ;
  }
}
