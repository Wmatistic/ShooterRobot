package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
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

        if (driver.wasJustPressed(GamepadKeys.Button.A)) {
            robot.shooter.setShooterState(Shooter.ShooterState.AIMING);
        }
        if (driver.wasJustPressed(GamepadKeys.Button.B)) {
            robot.shooter.setShooterState(Shooter.ShooterState.STOWED);
        }

        if (driver.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
            robot.intake.setIntakeState(Intake.IntakeState.INTAKING);
        } else {
            robot.intake.setIntakeState(Intake.IntakeState.STOWED);
        }


    }
}
