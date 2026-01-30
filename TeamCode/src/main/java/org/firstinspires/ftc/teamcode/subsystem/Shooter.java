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
        STOWED, AIMING, FIRING, FULL_SPEED, HUMAN_PLAYER
    }

    private double turretTarget;
    private double prevTurretPosition, turretTurns;
    private double turretOffset;

    private double localFarShotHood = 0.0;
    private double localFarShotRPM = 0.0;

    public Shooter() {
        this.robot = RobotHardware.getInstance();

        shooterControl = new ShooterControl();

        shooterState = ShooterState.STOWED;

        turretTarget = RobotConstants.Shooter.turretStowed;
        turretOffset = 0.0;

        prevTurretPosition = getRealTurretPosition();
        turretTurns = 0.0;
    }

    public void periodic() {
        switch (shooterState) {
            case STOWED:
                setFlywheelVelocity(0.0);
                robot.flywheel.stopMotor();
                setHoodPosition(RobotConstants.Shooter.hoodStowed);
                setTurretTarget(RobotConstants.Shooter.turretStowed);
                updateTurret();
                break;

            case FIRING:
            case AIMING:
                shooterControl.aimAndSpinSOTM();
//                setTurretTarget(shooterControl.findTurretToGoal());
//                setHoodPosition(localFarShotHood);
//                setFlywheelVelocity(localFarShotRPM);

                updateTurret();
                break;
            case FULL_SPEED:
                robot.flywheel.set(1.0);
                setHoodPosition(RobotConstants.Shooter.hoodMaxAngle);
                setTurretTarget(RobotConstants.Shooter.turretStowed);
                updateTurret();
                break;
            case HUMAN_PLAYER:
                robot.flywheel.set(RobotConstants.Shooter.flywheelHumanPlayer);
                setHoodPosition(RobotConstants.Shooter.hoodStowed);
                setTurretTarget(RobotConstants.Shooter.turretStowed);
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
        if (Double.isFinite(ticksPerSecond)) {
            robot.flywheel.set(ticksPerSecond);
        }
        //robot.shooterMotorLeft.setVelocity(ticksPerSecond);
        //robot.shooterMotorRight.setVelocity(ticksPerSecond);
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
        double position = getRealTurretPosition();

        if (prevTurretPosition - position > 0.5) {
            turretTurns++;
        } else if (prevTurretPosition - position < -0.5) {
            turretTurns--;
        }

        prevTurretPosition = position;

        return position + turretTurns;
    }

    public double getRealTurretPosition() {
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
    public void changeFarShotRPM(double value) {
        localFarShotRPM += value;
    }

    public double getLocalFarShotHood() {
        return localFarShotHood;
    }
    public double getLocalFarShotRPM() {
        return localFarShotRPM;
    }
}
