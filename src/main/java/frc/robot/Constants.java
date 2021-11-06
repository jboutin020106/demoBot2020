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
    public static final class DriveConstants 
    {
        // Drive Motor Constants
        public static final int FRONT_RIGHT_MOTOR = 3;
        public static final int BACK__RIGHT_MOTOR = 1;
        public static final int FRONT_LEFT_MOTOR = 4;
        public static final int BACK_LEFT_MOTOR = 2;

        // Current Limit Constant
        public static final int CURRENT_LIMIT = 60;

        // Voltage Comp Constant
        public static final double VOLTAGE_COMP = 12.0;

        //Ramp Rate Constant
        public static final double RAMP_RATE = 0.2;

        // Drive PID Constants
        public static final double kPDriveVel = 0.00289;
        public static final double kDDriveVel = 0.0;
        public static final double KP = kPDriveVel;
        public static final double KI = 0.0;
        public static final double KD = kDDriveVel;
        public static final double KIZ = 0.0;
        public static final double KFF = 0.0; 
        public static final int K_MAX_OUTPUT = 1;
        public static final int K_MIN_OUTPUT = -1;
        public static final int MAX_RPM = 5700;

    }
    
    public static final class IntakeAndOuttakeConstants
    {   
        //Intake and Outtake constants
        public static final int SHOOTER_LEADER = 8;
        public static final int SHOOTER_FOLLOWER = 9;
        public static final int CONVEYOR_MOTOR_1 = 6;
        public static final int CONVEYOR_MOTOR_2 = 7;
        public static final int INTAKE_PRIMARY = 5;
        public static final int INTAKE_SENSOR_1 = 1;
        public static final int INTAKE_SENSOR_2 = 2;
        public static final int INTAKE_SENSOR_3 = 3;

         // Current Limit Constant
         public static final int CURRENT_LIMIT = 60;

         // Voltage Comp Constant
         public static final double VOLTAGE_COMP = 12.0;
 
         //Ramp Rate Constant
         public static final double RAMP_RATE = 0.2;

        // PID CONSTANTS
         public static final double KP = 2.5e-4; 
        public static final double KD = 0.0;
        public static final int KIZ = 0;
        public static final double KFF = 0.0001732;
         public static final int K_MAX_OUTPUT = 1;
        public static final int K_MIN_OUTPUT = -1;
         public static final int MAXRPM = 5700;
    }
}
