/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.logging.Logger;

/**
 * An example command that uses an example subsystem.
 */
public class autoCloseShoot extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  int i;
  int timeRun;
  double speed;
  double targetSpeed;
  private static final Logger LOGGER = Logger.getLogger(autoShoot.class.getName());


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public autoCloseShoot(double speed, double targetSpeed,int timeRun) {
    this.speed=speed;
    this.timeRun = timeRun;
    this.targetSpeed = targetSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.mShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    i=0;
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    i++;
    RobotContainer.mShooterSubsystem.setVelocitySetpoint(speed*Constants.maxRPMShooter);
    RobotContainer.mShooterSubsystem.armShooter(targetSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.light.setValue(Constants.LL_LIGHT_OFF);

      RobotContainer.mShooterSubsystem.setSpeed(0);
        ShooterSubsystem.shooterReady = false;
       // LOGGER.warning("AUTO SHOOT DONE");
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((i*.02) >=timeRun);
  }
}
