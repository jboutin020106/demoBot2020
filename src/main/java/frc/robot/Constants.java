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
        private static final double kPDriveVel = 0.00289;
        private static final double kDDriveVel = 0.0;
        private static final double KP = kPDriveVel;
        private static final double KI = 0.0;
        private static final double KD = kDDriveVel;
        private static final double KIZ = 0.0;
        private static final double KFF = 0.0; 
        private static final int K_MAX_OUTPUT = 1;
        private static final int K_MIN_OUTPUT = -1;
        private static final int MAX_RPM = 5700;

    }
}
