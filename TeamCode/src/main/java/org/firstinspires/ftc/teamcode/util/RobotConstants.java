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

        public static double p = 0.0;
        public static double i = 0.0;
        public static double d = 0.0;
        public static double f = 0.0;
    }
}
