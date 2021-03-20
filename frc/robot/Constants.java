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
    
    public static  double K_V = 0.05;
    public static  double K_S = 0.22;
    public static final double K_A = 0.01;
    public static final double K_P = 0.2;
    public static final double K_I = 0.0;
    public static final double K_D = 0.001;

    // RAMSET
    public static final double K_RAMSET_B = 2;
    public static final double K_RAMSET_ZETA = 0.7;
        
    // Example value only - as above, this must be tuned for your drive!
    public static final double MAX_VELOCITY = 4.0;
    public static final double MAX_ACCELERATION = 4.0;
    
    public static final double WHEEL_BASE = 0.60;

    // Motors
    public static final int Left1Motor = 1;
    public static final int Left2Motor = 2;
    public static final int Right1Motor = 3;
    public static final int Right2Motor = 4;
    

    // Vision Interface
    public static final String BallAngleString = "Ball Angle";
    public static final String BallDistanceString = "Ball Distance"; 
    public static final double BallCameraToCenterDiatance = 0.25;   
    public static final double BallCameraToCenterAngle = 75;   

}
