/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class autoDeCorrect extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  double tx = SmartDashboard.getNumber("LimelightX", 0);
  double ty = SmartDashboard.getNumber("LimelightY", 0);
  double ta = SmartDashboard.getNumber("LimelightArea", 0);
  double d = 0;
  
  double minSteerAdjust = .2;
  double steeringAdjust = 0.0;
  double headingCommand = 0;
  double p = .013;
  double targetValue = 0;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public autoDeCorrect() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.light.setValue(Constants.LL_LIGHT_ON);
    targetValue = SmartDashboard.getNumber("Data Offset", 0);
    tx = SmartDashboard.getNumber("LimelightX", 0);
    ty = SmartDashboard.getNumber("LimelightY", 0);
    ta = SmartDashboard.getNumber("LimelightArea", 0);
    d = 73.5/Math.tan(Math.toRadians(ty+63));
    SmartDashboard.putNumber("Distance", d);
    if(ta != 0){
        if(tx>(targetValue +1)){
           steeringAdjust = p * (tx-targetValue) +minSteerAdjust;
        }
        else if(tx<(targetValue -1)){
           steeringAdjust = p * (tx-targetValue) -minSteerAdjust;
        }
    RobotContainer.mDriveSubsystem.arcadeDrive(0,steeringAdjust);
}

    steeringAdjust = 0;
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.light.setValue(Constants.LL_LIGHT_OFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(((tx - targetValue) < 1 && (tx - targetValue) > -1) || ta == 0 ){
        return true;
    }
    return false;
}
}
