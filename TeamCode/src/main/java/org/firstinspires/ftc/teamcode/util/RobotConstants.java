package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

public class RobotConstants {

    @Config
    public static class Drivetrain {
        public static String leftFront = "leftFront";
        public static String leftRear = "leftRear";
        public static String rightFront = "rightFront";
        public static String rightRear = "rightRear";
    }

    @Config
    public static class Shooter {
        public static String shooterMotorLeft = "shooterMotorLeft";
        public static String shooterMotorRight = "shooterMotorRight";
        public static String shooterServoLeft = "shooterServoLeft";
        public static String shooterServoRight = "shooterServoRight";
        public static String turretMotor = "turretMotor";
        public static String turretServoInput = "turretServoInput";

        public static double flywheelP = 0.0;
        public static double flywheelI = 0.0;
        public static double flywheelD = 0.0;
        public static double flywheelF = 0.0;

        public static double TICKS_PER_REV = 537.7;

        public static double turretStowed = 0.0;

        public static double turretP = 0.0;
        public static double turretI = 0.0;
        public static double turretD = 0.0;
        public static double turretF = 0.0;
    }
}
