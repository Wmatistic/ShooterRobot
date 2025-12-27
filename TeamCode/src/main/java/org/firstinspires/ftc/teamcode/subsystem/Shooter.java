package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.RobotHardware;
import org.firstinspires.ftc.teamcode.util.shooter.ShooterControl;

public class Shooter implements Subsystem {
    private final RobotHardware robot;

    public ShooterControl shooterControl;

    private ShooterState shooterState;
    public enum ShooterState {
        STOWED, AIMING, FIRING
    }

    private double turretTarget;
    private double turretOffset;

    private double localFarShotHood = 0.8;
    private int localFarShotRPM = 1500;

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
                //setFlywheelVelocity(0);
                setHoodPosition(RobotConstants.Shooter.hoodStowed);
                setTurretTarget(RobotConstants.Shooter.turretStowed);
                break;

            case FIRING:
            case AIMING:
                // TODO: maybe condense into something like Aim() method
                shooterControl.aimAndSpin();
//                setTurretTarget(shooterControl.findTurretToGoal());
//                setHoodPosition(localFarShotHood);
//                setFlywheelVelocity(localFarShotRPM);

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
        robot.shooterMotorLeft.setVelocity(ticksPerSecond);
        robot.shooterMotorRight.setVelocity(ticksPerSecond);
    }

    public void setHoodPosition(double hoodPosition) {
        robot.shooterServoRight.setPosition(Math.max(RobotConstants.Shooter.hoodMinAngle, Math.min(RobotConstants.Shooter.hoodMaxAngle, hoodPosition)));
        robot.shooterServoLeft.setPosition(Math.max(RobotConstants.Shooter.hoodMinAngle, Math.min(RobotConstants.Shooter.hoodMaxAngle, hoodPosition)));
    }

    public void updateTurret() {
        double correction = 0.0;

        if (!Double.isNaN(getTurretPosition()) && !Double.isNaN(turretTarget)) {
            correction = robot.turretPID.calculate(getTurretPosition(), turretTarget);
        }


//        if (Math.abs(correction) < RobotConstants.Shooter.turretBasePower && Math.abs(correction) > 0.05) {
//            if (correction < 0) {
//                robot.turretMotor.setPower(correction - RobotConstants.Shooter.turretBasePower);
//            } else {
//                robot.turretMotor.setPower(correction + RobotConstants.Shooter.turretBasePower);
//            }
//        } else {
//            robot.turretMotor.setPower(correction);
//        }

        robot.turretMotor.setPower(correction);
    }

    public double getTurretPosition() {
        return robot.turretServoInput.getVoltage() / 3.3;
    }

    public void setTurretTarget(double turretTarget) {
        this.turretTarget = Math.max(RobotConstants.Shooter.turretMinAngle, Math.min(RobotConstants.Shooter.turretMaxAngle, turretTarget));
    }

    public double getTurretTarget() {
        return turretTarget;
    }

    public void setTurretOffset(double turretOffset) {
        this.turretOffset = turretOffset;
    }

    public void changeFarShotHood(double value) {
        localFarShotHood += value;
    }
    public void changeFarShotRPM(int value) {
        localFarShotRPM += value;
    }

    public double getLocalFarShotHood() {
        return localFarShotHood;
    }
    public int getLocalFarShotRPM() {
        return localFarShotRPM;
    }
}
