package org.firstinspires.ftc.teamcode.util.shooter;

import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class ShooterControl {
    // TODO:
    //  * calculate distance to target from robot localization robot pose
    //  * calculate angle for turret to face the goal
    private final RobotHardware robot;
    private final ShooterLUT lut;
    private double distanceToTarget;
    private double turretGoal;

    public ShooterControl() {
        this.robot = RobotHardware.getInstance();
        lut = new ShooterLUT();
    }

    public void aimAndSpin() {
        ShotConfig config = lut.getForDistance(distanceToTarget);
        double ticksPerSecond = config.rpm * RobotConstants.Shooter.TICKS_PER_REV / 60.0;

        robot.shooter.setFlywheelVelocity(ticksPerSecond);
        robot.shooter.setHoodPosition(config.hoodPos);
        robot.shooter.setTurretTarget(turretGoal);
        robot.shooter.setTurretOffset(config.turretOffset);
    }

    public void findTurretGoal() {
        
    }
}
