package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class Drivetrain implements Subsystem {
    private RobotHardware robot;

    private GamepadEx driver;

    private double y, x, rx, leftFrontPower, leftRearPower, rightFrontPower, rightRearPower, heading, rotX, rotY;

    public Drivetrain() {
        this.robot = RobotHardware.getInstance();
    }

    public void setDriver(GamepadEx driver) {
        this.driver = driver;
    }

    public void periodic() {
        y = driver.getLeftY() * 1.2;
        x = driver.getLeftX() * 1.2;
        rx = driver.getRightX() * 1.1;

        leftFrontPower = (y + x + rx);
        leftRearPower = (y - x + rx);
        rightFrontPower = (y - x - rx);
        rightRearPower = (y + x - rx);

        robot.leftFront.setPower(leftFrontPower);
        robot.leftRear.setPower(leftRearPower);
        robot.rightFront.setPower(rightFrontPower);
        robot.rightRear.setPower(rightRearPower);
    }
}
