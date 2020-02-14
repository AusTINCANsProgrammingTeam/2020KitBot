/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import java.util.logging.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Add your docs here.
 */
public class ShooterSubsystem extends SubsystemBase{
    private static CANSparkMax mShoot;
    private CANPIDController pidController;
    private CANEncoder encoder;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
    private DoubleSolenoid solenoid; 
    private static final Logger LOGGER = Logger.getLogger(DriveSubsystem.class.getName());
    private double m_setpoint = 0;




    public ShooterSubsystem() {
        mShoot = new CANSparkMax(1, MotorType.kBrushless);
        mShoot.restoreFactoryDefaults();
        mShoot.enableVoltageCompensation(12);
        mShoot.setIdleMode(IdleMode.kBrake);

        pidController = mShoot.getPIDController();
        encoder = mShoot.getEncoder();

        kP = 0.00010; 
        kI = 0;
        kD = .0000; 
        kIz = 0; 
        kFF = 0.000175; 
        kMaxOutput = 1; 
        kMinOutput = -1;

        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setIZone(kIz);
        pidController.setFF(kFF);
        pidController.setOutputRange(kMinOutput, kMaxOutput);
        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("FF Value", kFF);
    }
    public void updatePID(){
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);
      
        //if PID coefficients on SmartDashboard have changed, write new values to controller
        if((p != kP)) { pidController.setP(p); kP = p; 
        LOGGER.warning("PID CHANGED");}
        if((i != kI)) { pidController.setI(i); kI = i; 
        LOGGER.warning("PID CHANGED");}
        if((d != kD)) { pidController.setD(d); kD = d; 
        LOGGER.warning(pidController.getD() +" D CHANGED");}
        if((max != kMaxOutput) || (min != kMinOutput)) 
        { 
            // m_pidController.setOutputRange(min, max); 
            // kMinOutput = min;
            // kMaxOutput = max; 
        }
    }
        public void setPIDVelocitySetpoint(double setpoint)
        {
            pidController.setReference(setpoint, ControlType.kVelocity);
        }
        public double velocity()
        {
            return encoder.getVelocity();
        }
        public double fpsToRPM(double fps){
            fps = fps * 12;
            fps = fps/Constants.kWheelCircumference;
            fps = fps *60;
            fps = fps*Constants.kGearRatio;
            return fps;
        }

        /*
        *   Controls the engagement of the shooter.
        *   @state kReverse disengages shooter, kForward engages shooter, kOff locks shooter in its current position
        */
        public void setDoubleSolenoidState(DoubleSolenoid.Value state) {
            solenoid.set(state); //kOff, kReverse, kForward
        }
        public void toggleSolenoidState() {
            switch(solenoid.get()) {
                case kForward:
                    setDoubleSolenoidState(DoubleSolenoid.Value.kReverse);
                    break;
                case kReverse:
                    setDoubleSolenoidState(DoubleSolenoid.Value.kForward);
                    break;
                case kOff:
                    setDoubleSolenoidState(DoubleSolenoid.Value.kForward);
                    break;
            }
        }

        //sets the SparkMax Motor's speed setpoint to 1
        public static void m_forward() { 
            mShoot.set(1);
        }

        //sets the SparkMax Motor's speed setpoint to 0
        public static void m_stop() {
            mShoot.set(0);
        }

        //sets the SparkMax Motor's speed setpoint to -1
        public static void m_backward() {
            mShoot.set(-1);
        }

        public void m_toggle() {
            if(m_setpoint==0)
                m_setpoint = 1;
            else
                m_setpoint = 0;
            mShoot.set(m_setpoint);
        }

          @Override
          public void periodic() {
            // This method will be called once per scheduler run
        
          }
}