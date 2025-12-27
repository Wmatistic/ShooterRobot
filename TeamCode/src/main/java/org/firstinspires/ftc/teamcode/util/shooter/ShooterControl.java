package org.firstinspires.ftc.teamcode.util.shooter;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class ShooterControl {
    // TODO:
    //  * calculate distance to target from robot localization robot pose
    //  * calculate angle for turret to face the goal
    private final RobotHardware robot;
    private final ShooterLUT lut;
    private double distanceToGoal;
    private double turretToGoal;

    public ShooterControl() {
        this.robot = RobotHardware.getInstance();
        lut = new ShooterLUT();
    }

    public void aimAndSpin() {
        distanceToGoal = findDistanceToGoal();
        ShotConfig config = lut.getForDistance(distanceToGoal);
        turretToGoal = findTurretToGoal();

        robot.shooter.setFlywheelVelocity(config.rpm);
        robot.shooter.setHoodPosition(config.hoodPos);
        robot.shooter.setTurretTarget(turretToGoal);
        robot.shooter.setTurretOffset(config.turretOffset);
    }

    public double findDistanceToGoal() {
        Pose2d goalPos = RobotConstants.RobotLocalization.goalPos;
        Pose2d robotPose = robot.robotLocalization.getRobotPose();

        double dx = goalPos.position.x - robotPose.position.x;
        double dy = goalPos.position.y - robotPose.position.y;

        return Math.hypot(dx, dy);
    }

    public double findTurretToGoal() {
        Pose2d goalPos = RobotConstants.RobotLocalization.goalPos;
        Pose2d robotPose = robot.robotLocalization.getRobotPose();

        double dx = goalPos.position.x - robotPose.position.x;
        double dy = goalPos.position.y - robotPose.position.y;

        double targetAngle = Math.atan2(dy, dx);

        double turretAngle = targetAngle - robotPose.heading.log();

        return (((normalizeAngle(turretAngle) / Math.PI) + 1) / 2.0) + RobotConstants.Shooter.turretOffset;
    }

    public static double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
