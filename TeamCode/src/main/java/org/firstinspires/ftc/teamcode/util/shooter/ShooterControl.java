package org.firstinspires.ftc.teamcode.util.shooter;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;

import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class ShooterControl {
    private final RobotHardware robot;
    private final ShooterLUT lut;
    private double distanceVelCompensation;
    private double turretVelCompensation;

    private double goalXOffset, goalYOffset;

    public ShooterControl() {
        this.robot = RobotHardware.getInstance();
        lut = new ShooterLUT();
    }

    public void aimAndSpin() {
        double distanceToGoal = findDistanceToGoal();
        ShotConfig config = lut.getForDistance(distanceToGoal);
        double turretToGoal = findTurretToGoal();

        robot.shooter.setFlywheelVelocity(config.rpm);
        robot.shooter.setHoodPosition(config.hoodPos);
        robot.shooter.setTurretTarget(turretToGoal);
        robot.shooter.setTurretOffset(config.turretOffset);
    }

    public void aimAndSpinSOTM() {
        Pose2d goalPos = RobotConstants.RobotLocalization.goalPos;
        goalPos = new Pose2d(
                goalPos.position.x + goalXOffset,
                goalPos.position.y + goalYOffset,
                goalPos.heading.real
        );
        Pose2d robotPose = robot.robotLocalization.getRobotPose();
        PoseVelocity2d robotVel = robot.robotLocalization.getRobotVelocity();

        double dx = goalPos.position.x - robotPose.position.x;
        double dy = goalPos.position.y - robotPose.position.y;

        double dist = Math.hypot(dx, dy);
        if (dist < 1e-6) dist = 1e-6; // avoid divide by zero

        // Unit vector toward goal (world frame)
        double ux = dx / dist;
        double uy = dy / dist;

        // Robot translational velocity in world frame
        double vx = robotVel.linearVel.x;
        double vy = robotVel.linearVel.y;

        // Decompose velocity relative to goal line
        double vAlong = vx * ux + vy * uy;           // + toward goal
        double uPerpX = -uy;                          // left of u
        double uPerpY = ux;
        double vLat = vx * uPerpX + vy * uPerpY;      // + left relative to goal line

        // Baseline LUT from current distance
        ShotConfig base = lut.getForDistance(dist);

        // Estimate time-of-flight
        double tof = estimateTofSec(dist);

        // Effective distance adjustment
        double distAdj = clamp(vAlong * tof, -RobotConstants.Shooter.MAX_DIST_ADJ, RobotConstants.Shooter.MAX_DIST_ADJ);
        double effectiveDist = Math.max(0.0, dist - distAdj);

        // Re-sample LUT at effective distance
        ShotConfig config = lut.getForDistance(effectiveDist);

        // Lead angle for lateral motion: atan2(lateral displacement, range)
        // lateral displacement = vLat * tof
        double leadRad = Math.atan2(vLat * tof, Math.max(1e-6, effectiveDist));
        leadRad = clamp(leadRad, -RobotConstants.Shooter.MAX_LEAD_RAD, RobotConstants.Shooter.MAX_LEAD_RAD);

        // Bearing robot->goal in world frame
        double targetAngleWorld = Math.atan2(dy, dx);

        // Desired turret direction in world frame, then convert to robot-relative
        double turretAngleRobot = (targetAngleWorld - leadRad) - robotPose.heading.log();
        turretAngleRobot = normalizeAngle(turretAngleRobot);

        // Convert robot-relative turret angle (rad) -> servo target [0..1] with offset
        double turretTarget = (((turretAngleRobot / Math.PI) + 1) / 2.0) + RobotConstants.Shooter.turretOffset;

        // Debug
        distanceVelCompensation = effectiveDist - dist;
        turretVelCompensation = leadRad;

        robot.shooter.setFlywheelVelocity(config.rpm);
        robot.shooter.setHoodPosition(config.hoodPos);
        robot.shooter.setTurretTarget(turretTarget);
        robot.shooter.setTurretOffset(config.turretOffset);
    }

    public double getDistanceVelCompensation() {
        return distanceVelCompensation;
    }

    public double getTurretVelCompensation() {
        return turretVelCompensation;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double estimateTofSec(double distance) {
        double tof = distance * RobotConstants.Shooter.DEFAULT_TOF_PER_IN;
        return clamp(tof, RobotConstants.Shooter.MIN_TOF_SEC, RobotConstants.Shooter.MAX_TOF_SEC);
    }

    public double findDistanceToGoal() {
        Pose2d goalPos = RobotConstants.RobotLocalization.goalPos;
        goalPos = new Pose2d(goalPos.position.x + goalXOffset, goalPos.position.y + goalYOffset, goalPos.heading.real);
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

    public void changeGoalXOffset(double x) {
        goalXOffset += x;
    }

    public void changeGoalYOffset(double y) {
        goalYOffset += y;
    }

    public double getGoalXOffset() {
        return goalXOffset;
    }

    public double getGoalYOffset() {
        return goalYOffset;
    }
}
