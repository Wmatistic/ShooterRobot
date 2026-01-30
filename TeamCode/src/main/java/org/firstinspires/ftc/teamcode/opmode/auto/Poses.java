package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

public class Poses {

    @Config
    public static class BlueAuto {
        public static Pose2d start = new Pose2d(49, 53, Math.toRadians(-45));

        public static Pose2d firstShot = new Pose2d(10, 10, Math.toRadians(90));

        public static Pose2d intakeFirstLine = new Pose2d(10, 52, Math.toRadians(90));
        public static Pose2d shootFirstLine = new Pose2d(10,20, Math.toRadians(90));

        public static Pose2d intakeSecondLineInitial = new Pose2d(-14.5, 30, Math.toRadians(90));
        public static Pose2d intakeSecondLinePost = new Pose2d(-14.5, 64, Math.toRadians(90));
        public static Pose2d shootSecondLine = new Pose2d(10, 20, Math.toRadians(180));

        public static Pose2d intakeThirdLineInitial = new Pose2d(-32, 30, Math.toRadians(90));
        public static Pose2d intakeThirdLinePost = new Pose2d(-32, 64, Math.toRadians(90));
        public static Pose2d shootThirdLine = new Pose2d(10, 20, Math.toRadians(180));
    }
}
