package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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
    PoseVelocity2d robotVelocity;

    public RobotLocalization() {
        this.robot = RobotHardware.getInstance();

        limeLightPose = new Pose2d(0, 0, 0);
        robotPose = new Pose2d(0, 0, 0);
        pinpointPose = RobotConstants.RobotLocalization.start;
        setPinpointPose(RobotConstants.RobotLocalization.start);
    }

    public void periodic() {
        setLimelightPose();
        updatePinpointPose();
        updateRobotVelocity();

        robotPose = pinpointPose;
    }

    public void setLimelightPose() {
        result = robot.limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                limeLightPose = new Pose2d(-1 * botpose.getPosition().x * 39.370079, -1 * botpose.getPosition().y * 39.370079, Math.toRadians(((360 - (-1 * robot.limelight.getLatestResult().getBotpose().getOrientation().getYaw(AngleUnit.DEGREES))) % 360) - 180));
            }
        }
    }

    public Pose2d getLimelightPose() {
        return limeLightPose;
    }

    public void updatePinpointPose() {
        robot.pinpointDrive.updatePoseEstimate();
        pinpointPose = new Pose2d(robot.pinpointDrive.pinpoint.getPosition().getX(DistanceUnit.INCH), robot.pinpointDrive.pinpoint.getPosition().getY(DistanceUnit.INCH), robot.pinpointDrive.pinpoint.getHeading());
    }

    public void updateRobotVelocity() {
        robotVelocity = robot.pinpointDrive.pinpoint.getVelocityRR();
    }

    public PoseVelocity2d getRobotVelocity() {
        return robotVelocity;
    }

    public void relocalizePinpointWithLimelight() {
        robotPose = limeLightPose;
        robot.pinpointDrive.pinpoint.setPosition(limeLightPose);
        pinpointPose = limeLightPose;
        robot.pinpointDrive.updatePoseEstimate();
    }

    public void setPinpointPose(Pose2d pose) {
        robot.pinpointDrive.pinpoint.setPosition(pose);
    }

    public Pose2d getRobotPose() {
        return robotPose;
    }
}
