package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class RobotLocalization {
    private final RobotHardware robot;

    Pose2d robotPose;

    // Limelight
    LLResult result;
    Pose2d limeLightPose;

    // Pinpoint
    Pose2d pinpointPose;

    public RobotLocalization() {
        this.robot = RobotHardware.getInstance();

        limeLightPose = new Pose2d(0, 0, 0);
        pinpointPose = new Pose2d(0, 0, 0);
    }

    public void periodic() {
        //setLimelightPose();
        setPinpointPose();

        // TODO: average the two poses somehow when limelight pose is valid
        robotPose = pinpointPose;
    }

    public void setLimelightPose() {
        double robotYaw = robot.imu.getRobotYawPitchRollAngles().getYaw();
        robot.limelight.updateRobotOrientation(robotYaw);
        if (result != null && result.isValid()) {
            Pose3D botpose_mt2 = result.getBotpose_MT2();
            if (botpose_mt2 != null) {
                limeLightPose = new Pose2d(botpose_mt2.getPosition().x, botpose_mt2.getPosition().y, Math.toRadians(0));
            }
        }
    }

    public void setPinpointPose() {
        robot.pinpointDrive.updatePoseEstimate();
        pinpointPose = new Pose2d(robot.pinpointDrive.pinpoint.getPosition().getX(DistanceUnit.INCH), robot.pinpointDrive.pinpoint.getPosition().getY(DistanceUnit.INCH), robot.pinpointDrive.pinpoint.getHeading());
    }

    public Pose2d getRobotPose() {
        return robotPose;
    }
}
