package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

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

        public static double flywheelP = 100;
        public static double flywheelI = 0.0;
        public static double flywheelD = 0.0;
        public static double flywheelF = 0.0;

        public static double TICKS_PER_REV = 537.7;

        public static double turretStowed = 0.52;
        public static double turretOffset = 0.02;
        public static double turretBasePower = 0.25;

        public static double turretP = 10.0;
        public static double turretI = 10.0;
        public static double turretD = 0.0;
        public static double turretF = 0.0;

        public static double turretMinAngle = 0.1;
        public static double turretMaxAngle = 0.6;

        public static double hoodStowed = 0.0;
        public static double hoodMinAngle = 0.0;
        public static double hoodMaxAngle = 1.0;

        public static int farShotRPM = 1400;
        public static double farShotHood = 0.8;
    }

    @Config
    public static class Intake {
        public static String intakeMotor = "intakeMotor";

        public static double intakePower = 1.0;
        public static double intakeOff = 0.0;
        public static double intakeReverse = -1.0;
    }

    @Config
    public static class Indexer {
        public static String frontLeftSensor = "frontLeftSensor";
        public static String frontRightSensor = "frontRightSensor";
        public static String middleLeftSensor = "frontLeftSensor";
        public static String middleRightSensor = "middleRightSensor";
        public static String rearLeftSensor = "rearLeftSensor";
        public static String rearRightSensor = "rearRightSensor";
        public static String frontServo = "frontServo";
        public static String middleServo = "middleServo";
        public static String rearServo = "rearServo";

        public static double proximityThreshold = 25.0;

        public static double indexerServoStowed = 0.1;
        public static double frontIndexerServoStowed = 0.01;
        public static double middleIndexerServoStowed = 0.09;
        public static double rearIndexerServoStowed = 0.068;
        public static double indexerServoUp = 0.5;
        public static double frontIndexerServoUp = 0.5;
        public static double middleIndexerServoUp = 0.6;
        public static double rearIndexerServoUp = 0.4;
    }

    @Config
    public static class RobotLocalization {
        public static Pose2d start = new Pose2d(0, 0, Math.toRadians(0));

        public static Pose2d goalPos = new Pose2d(66, 66, 0);
    }
}
