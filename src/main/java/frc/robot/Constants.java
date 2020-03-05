/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int kNumSensorUnitsPerRotation = 4096;
    public static final double kTurnTravelUnitsPerRotation = 3600;
    public static final int kEncoderUnitsPerRotation = 51711;
    public static final int kTimeOutMs = 10;
    public static final double kNeutralDeadband = 0.001;
    public static final double kMaxRpms = 5700;
    public static final double kGearRatio = 10.71;
    public static final double kWheelCircumference = Math.PI*5.9;

    //public final static Gains kGains_Velocity 
    //= new Gains(5e-5, 0, 0.0, 0, 1, -1);
    
    public final static double kPDrive = 0.000600;
    public final static double kIDrive = 0;
    public final static double kDDrive = 0;
    public final static double kIzDrive = 0;
    public final static double kFFDrive = 0.000000;
    public final static double kMaxOutputDrive = 1;
    public final static double kMinOutputDrive = -1;

    public final static int LL_LIGHT_DEFAULT = 0;
    public final static int LL_LIGHT_OFF = 1;
    public final static int LL_LIGHT_BLINK = 2;
    public final static int LL_LIGHT_ON = 3;
//     6     13 14  16
    public final static int DriveLeft1 = 1;
    public final static int DriveLeft2 = 2;
    public final static int DriveLeft3 = 16;
    public final static int DriveRight1 = 15;
    public final static int DriveRight2 = 14;
    public final static int DriveRight3 = 13;
    public final static int Shooter = 10;
    public final static int Intake = 5;
    public final static int Hopper1 = 4;
    public final static int Hopper2 = 11;
    public final static int Elevator1 = 12;
    public final static int Elevator2 = 3;
    public final static int Belt = 9;    


    public final static double maxRPMShooter = 5676;

}

