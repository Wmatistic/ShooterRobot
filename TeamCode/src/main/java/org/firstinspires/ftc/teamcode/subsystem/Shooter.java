package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.RobotHardware;
import org.firstinspires.ftc.teamcode.util.shooter.ShooterControl;

public class Shooter implements Subsystem {
    private final RobotHardware robot;

    private ShooterControl shooterControl;

    private ShooterState shooterState;
    public enum ShooterState {
        STOWED, AIMING, FIRING
    }

    private double turretTarget;
    private double turretOffset;

    public Shooter() {
        this.robot = RobotHardware.getInstance();

        shooterControl = new ShooterControl();

        shooterState = ShooterState.STOWED;

        turretTarget = RobotConstants.Shooter.turretStowed;
        turretOffset = 0.0;
    }

    public void periodic() {
        switch (shooterState) {
            case STOWED:
                setFlywheelVelocity(0);
                setHoodPosition(RobotConstants.Shooter.hoodStowed);
                setTurretTarget(RobotConstants.Shooter.turretStowed);
                break;

            case FIRING:
            case AIMING:
                // TODO: maybe condense into something like Aim() method
                shooterControl.aimAndSpin();

                updateTurret();
                break;
        }
    }

    public void setShooterState(ShooterState shooterState) {
        this.shooterState = shooterState;
    }

    public ShooterState getShooterState() {
        return shooterState;
    }

    public void setFlywheelVelocity(double ticksPerSecond) {
        robot.shooterMotorRight.setVelocity(ticksPerSecond);
        robot.shooterMotorLeft.setVelocity(ticksPerSecond);
    }

    public void setHoodPosition(double hoodPosition) {
        robot.shooterServoRight.setPosition(hoodPosition);
        robot.shooterServoLeft.setPosition(hoodPosition);
    }

    public void updateTurret() {
        double correction = robot.turretPID.calculate(getTurretPosition(), turretTarget + turretOffset);

        robot.turretMotor.setPower(correction);
    }

    public double getTurretPosition() {
        return robot.turretServoInput.getVoltage() / 3.3;
    }

    public void setTurretTarget(double turretTarget) {
        this.turretTarget = turretTarget;
    }

    public void setTurretOffset(double turretOffset) {
        this.turretOffset = turretOffset;
    }
}
