package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.teleopcommand.TransferBallCommand;
import org.firstinspires.ftc.teamcode.subsystem.Indexer;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

@TeleOp
public class Lebot extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    private GamepadEx driver, operator;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        robot.init(hardwareMap);
        robot.drivetrain.setDriver(driver);

        //robot.limelight.start();
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        robot.periodic();
        driver.readButtons();
        operator.readButtons();

        telemetry.addData("Flywheel Velocity: ", robot.shooterMotorLeft.getVelocity());
        telemetry.addData("Local Far Shot RPM: ", robot.shooter.getLocalFarShotRPM());
        telemetry.addData("Local Far Shot Hood: ", robot.shooter.getLocalFarShotHood());
        telemetry.addData("Left Motor Power: ", robot.shooterMotorLeft.getPower());
        telemetry.addData("Right Motor Power: ", robot.shooterMotorRight.getPower());
        telemetry.addData("Turret Position: ", robot.shooter.getTurretPosition());
        telemetry.addData("Turret to Goal: ", robot.shooter.shooterControl.findTurretToGoal());
        telemetry.addData("Turret Motor Power: ", robot.turretMotor.getPower());
        telemetry.addData("Turret Target: ", robot.shooter.getTurretTarget());
        telemetry.addData("Distance to Goal: ", robot.shooter.shooterControl.findDistanceToGoal());
        telemetry.update();

        if (driver.wasJustPressed(GamepadKeys.Button.A)) {
            robot.shooter.setShooterState(Shooter.ShooterState.AIMING);
        }
        if (driver.wasJustPressed(GamepadKeys.Button.B)) {
            robot.shooter.setShooterState(Shooter.ShooterState.STOWED);
        }

//        if (driver.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
//            robot.shooter.changeFarShotHood(-0.1);
//        }
//        if (driver.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
//            robot.shooter.changeFarShotHood(0.1);
//        }
//
//        if (driver.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
//            robot.shooter.changeFarShotRPM(100);
//        }
//        if (driver.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
//            robot.shooter.changeFarShotRPM(-100);
//        }

//        if (driver.isDown(GamepadKeys.Button.B)) {
//            robot.shooter.setFlywheelVelocity(RobotConstants.Shooter.farShotRPM);
//        } else {
//            robot.shooter.setFlywheelVelocity(0);
//        }

        if (driver.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
            robot.intake.setIntake(Intake.IntakeState.INTAKING);
        } else if (driver.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            robot.intake.setIntake(Intake.IntakeState.REVERSED);
        } else {
            robot.intake.setIntake(Intake.IntakeState.STOWED);
        }

        if (driver.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            CommandScheduler.getInstance().schedule(
                    new TransferBallCommand(Indexer.SlotID.FRONT)
            );
        }
        if (driver.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            CommandScheduler.getInstance().schedule(
                    new TransferBallCommand(Indexer.SlotID.MIDDLE)
            );
        }
        if (driver.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            CommandScheduler.getInstance().schedule(
                    new TransferBallCommand(Indexer.SlotID.REAR)
            );
        }

//        if (driver.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
//            robot.shooter.setTurretTarget(robot.shooter.getTurretTarget() + 0.05);
//        }
//        if (driver.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
//            robot.shooter.setTurretTarget(robot.shooter.getTurretTarget() - 0.05);
//        }
    }
}
