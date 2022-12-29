// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    
    //CAN IDs
    // 1: BR, 2: FL, 3: FR, 4: BL

    //cancoder
    public static final int c3 = 23;//FL
    public static final int c2 = 22;//FR
    public static final int c4 = 24;//BL
    public static final int c1 = 21;//BR

    //drive motor
    public static final int d3 = 15;
    public static final int d2 = 13;
    public static final int d4 = 17;
    public static final int d1 = 11;

    //steer motor
    public static final int s3 = 16;
    public static final int s2 = 14;
    public static final int s4 = 18;
    public static final int s1 = 12;

    public static final int pigeonPort = 25;

    //cancoder offsets (cancoder value when wheel is straight, [0, 360))
    public static final double c1offset = -6.4+180;
    public static final double c2offset = -56.9;
    public static final double c3offset = 25.1;
    public static final double c4offset = -265.4+360;

    public static final double turnP = 0.015d;
    public static final double turnD = 0;
    public static final double moveGain = 1;

    public static final double driveGain = 4000;

    public static final double triggerGain = 0.3;

    public static final int STEER_INVERT = 1; //1 = false, -1 = true
    public static final int DRIVE_INVERT = 1;

    public static final double deadband = 0.1;
    public static final double deadbandSteer = 0.5;

    public static final double STEER_P = 0.1;
    public static final double STEER_I = 0;
    public static final double STEER_D = 0;
    public static final double STEER_F = 0;


    // public static final double CANCODER_OFFSET_FRONT_LEFT = 359.0;
    // public static final double CANCODER_OFFSET_FRONT_RIGHT = 228.0; //318.0;
    // public static final double CANCODER_OFFSET_BACK_LEFT = 4.0;
    // public static final double CANCODER_OFFSET_BACK_RIGHT = 97.0; // 187.0; 

    public static final double STEER_GEAR_RATIO = 150 / 7;



    public static final double rampRateSteer = 1;
    public static final double statorCurrentLimitSteer = 20;
    public static final double rampRateDrive = 1;
    public static final double statorCurrentLimitDrive = 20;


    public static final int buttonY = 4;
}
