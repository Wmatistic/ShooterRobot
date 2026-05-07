package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.IndexerArmCommand;
import org.firstinspires.ftc.teamcode.commands.teleopcommand.FourBallShuffle;
import org.firstinspires.ftc.teamcode.commands.teleopcommand.TransferBallCommand;
import org.firstinspires.ftc.teamcode.subsystem.Indexer;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class Lebot extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    private GamepadEx driver, operator;

    private boolean transferActive, hasRumbled;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        robot.init(hardwareMap);
        robot.drivetrain.setDriver(driver);

        robot.robotLocalization.setPinpointPose(RobotConstants.RobotLocalization.start);

        robot.limelight.start();

        transferActive = false;
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        robot.periodic();
        driver.readButtons();
        operator.readButtons();

        List<Double> velocities = robot.flywheel.getVelocities();
        telemetry.addData("Left Flywheel Velocity: ", velocities.get(0));
        telemetry.addData("Right Flywheel Velocity: ", velocities.get(1));
        telemetry.addData("Effective Distance: ", robot.shooter.shooterControl.getDistanceVelCompensation());
        telemetry.addData("Effective Turret: ", robot.shooter.shooterControl.getTurretVelCompensation());
        telemetry.addData("Local Far Shot RPM: ", robot.shooter.getLocalFarShotRPM());
        telemetry.addData("Local Far Shot Hood: ", robot.shooter.getLocalFarShotHood());
        telemetry.addData("Turret Position: ", robot.shooter.getTurretPosition());
        telemetry.addData("Turret to Goal: ", robot.shooter.shooterControl.findTurretToGoal());
        telemetry.addData("Turret Motor Power: ", robot.turretMotor.getPower());
        telemetry.addData("Turret Target: ", robot.shooter.getTurretTarget());
        telemetry.addData("Distance to Goal: ", robot.shooter.shooterControl.findDistanceToGoal());
        telemetry.addData("Ball Count: ", robot.indexer.getBallCount());
        telemetry.addData("Goal X: ", RobotConstants.RobotLocalization.redGoalPos.position.x + robot.shooter.shooterControl.getGoalXOffset());
        telemetry.addData("Goal Y: ", RobotConstants.RobotLocalization.redGoalPos.position.y + robot.shooter.shooterControl.getGoalYOffset());
//        telemetry.addData("Limelight Pose Position: ", robot.robotLocalization.getLimelightPose().position.toString());
//        telemetry.addData("Limelight Pose Heading: ", ((360 - (-1 * robot.limelight.getLatestResult().getBotpose().getOrientation().getYaw(AngleUnit.DEGREES))) % 360) - 180);
//        telemetry.addData("Pinpoint Pose Position: ", robot.pinpointDrive.pinpoint.getPositionRR().position.toString());
//        telemetry.addData("Pinpoint Pose Heading: ", Math.toDegrees(robot.pinpointDrive.pinpoint.getHeading()));
        telemetry.update();

        if (driver.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            robot.shooter.setShooterState(Shooter.ShooterState.FULL_SPEED);
        }
        if (driver.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT) && robot.shooter.getShooterState() == Shooter.ShooterState.STOWED) {
            robot.shooter.setShooterState(Shooter.ShooterState.HUMAN_PLAYER);
        }

        if (driver.wasJustPressed(GamepadKeys.Button.BACK)) {
            if (robot.shooter.getShooterState() == Shooter.ShooterState.DEBUG)
                robot.shooter.setShooterState(Shooter.ShooterState.STOWED);
            else {
                robot.shooter.setShooterState(Shooter.ShooterState.DEBUG);
            }
        }

        if (robot.shooter.getShooterState() == Shooter.ShooterState.DEBUG) {
            if (driver.wasJustPressed(GamepadKeys.Button.A)) {
                robot.shooter.changeFarShotHood(-0.1);
            }
            if (driver.wasJustPressed(GamepadKeys.Button.Y)) {
                robot.shooter.changeFarShotHood(0.1);
            }

            if (driver.wasJustPressed(GamepadKeys.Button.B)) {
                robot.shooter.changeFarShotRPM(0.01);
            }
            if (driver.wasJustPressed(GamepadKeys.Button.X)) {
                robot.shooter.changeFarShotRPM(-0.01);
            }
        } else {
            if (driver.wasJustPressed(GamepadKeys.Button.A)) {
                robot.shooter.shooterControl.changeGoalXOffset(-0.5);
            }
            if (driver.wasJustPressed(GamepadKeys.Button.Y)) {
                robot.shooter.shooterControl.changeGoalXOffset(0.5);
            }
            if (driver.wasJustPressed(GamepadKeys.Button.B)) {
                robot.shooter.shooterControl.changeGoalYOffset(-0.5);
            }
            if (driver.wasJustPressed(GamepadKeys.Button.X)) {
                robot.shooter.shooterControl.changeGoalYOffset(0.5);
            }
        }

        if (driver.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            if (robot.shooter.getShooterState() == Shooter.ShooterState.STOWED) {
                robot.shooter.setShooterState(Shooter.ShooterState.AIMING);
            } else {
                robot.shooter.setShooterState(Shooter.ShooterState.STOWED);
            }
        }

        if (driver.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            robot.robotLocalization.relocalizePinpointWithLimelight();
        }

        if (driver.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
            robot.changeTeam();
        }

        if (driver.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
            robot.intake.setIntake(Intake.IntakeState.INTAKING);
        } else if (driver.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            robot.intake.setIntake(Intake.IntakeState.REVERSED);
            CommandScheduler.getInstance().schedule(
                    new FourBallShuffle()
            );
        } else {
            robot.intake.setIntake(Intake.IntakeState.STOWED);
        }

//        if (driver.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
//            CommandScheduler.getInstance().schedule(
//                    new TransferBallCommand(Indexer.SlotID.FRONT)
//            );
//        }
//        if (driver.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
//            CommandScheduler.getInstance().schedule(
//                    new TransferBallCommand(Indexer.SlotID.MIDDLE)
//            );
//        }
//        if (driver.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
//            CommandScheduler.getInstance().schedule(
//                    new TransferBallCommand(Indexer.SlotID.REAR)
//            );
//        }

        /*
        if (driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2 && (robot.shooter.getShooterState() == Shooter.ShooterState.AIMING || robot.shooter.getShooterState() == Shooter.ShooterState.FULL_SPEED)) {

            List<Indexer.SlotID> slotOrder = Arrays.asList(
                    Indexer.SlotID.MIDDLE,
                    Indexer.SlotID.REAR,
                    Indexer.SlotID.FRONT
            );

            int[] startOffsetsMs = new int[] { RobotConstants.Indexer.firstStartOffset, RobotConstants.Indexer.secondStartOffset, RobotConstants.Indexer.thirdStartOffset };

            List<Indexer.SlotID> occupied = new ArrayList<>();
            for (Indexer.SlotID slot : slotOrder) {
                if (robot.indexer.getIndexerSlot(slot).occupied) {
                    occupied.add(slot);
                }
            }

            if (!occupied.isEmpty()) {
                ParallelCommandGroup burst = new ParallelCommandGroup();

                int shots = Math.min(occupied.size(), startOffsetsMs.length);
                for (int i = 0; i < shots; i++) {
                    Indexer.SlotID slot = occupied.get(i);
                    int offset = startOffsetsMs[i];

                    burst.addCommands(
                            new SequentialCommandGroup(
                                    new WaitCommand(offset),
                                    new IndexerArmCommand(slot, RobotHardware.getInstance().indexer.getIndexerServoUpPosition(slot)),
                                    new WaitCommand(RobotConstants.Indexer.armHoldMs),
                                    new IndexerArmCommand(slot, RobotConstants.Indexer.indexerServoCorral)
                            )
                    );
                }

                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                burst,
                                new ParallelCommandGroup(
                                        new IndexerArmCommand(Indexer.SlotID.MIDDLE, RobotHardware.getInstance().indexer.getIndexerServoStowedPosition(Indexer.SlotID.MIDDLE)),
                                        new IndexerArmCommand(Indexer.SlotID.REAR, RobotHardware.getInstance().indexer.getIndexerServoStowedPosition(Indexer.SlotID.REAR)),
                                        new IndexerArmCommand(Indexer.SlotID.FRONT, RobotHardware.getInstance().indexer.getIndexerServoStowedPosition(Indexer.SlotID.FRONT))
                                )
                        )
                );
            }
        }
         */




        if (driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2 && !transferActive && (robot.shooter.getShooterState() == Shooter.ShooterState.AIMING || robot.shooter.getShooterState() == Shooter.ShooterState.FULL_SPEED || robot.shooter.getShooterState() == Shooter.ShooterState.DEBUG)) {
            transferActive = true;

            List<Boolean> occupiedList = new ArrayList<Boolean>();
            occupiedList.add(robot.indexer.getIndexerSlot(Indexer.SlotID.FRONT).occupied);
            occupiedList.add(robot.indexer.getIndexerSlot(Indexer.SlotID.MIDDLE).occupied);
            occupiedList.add(robot.indexer.getIndexerSlot(Indexer.SlotID.REAR).occupied);
//
//            if (!robot.indexer.getIndexerSlot(Indexer.SlotID.MIDDLE).occupied) {
//                robot.indexer.moveIndexerSlotServo(Indexer.SlotID.MIDDLE, RobotConstants.Indexer.indexerServoCorral);
//            }
//            if (!robot.indexer.getIndexerSlot(Indexer.SlotID.REAR).occupied) {
//                robot.indexer.moveIndexerSlotServo(Indexer.SlotID.REAR, RobotConstants.Indexer.indexerServoCorral);
//            }
//            if (!robot.indexer.getIndexerSlot(Indexer.SlotID.FRONT).occupied) {
//                robot.indexer.moveIndexerSlotServo(Indexer.SlotID.FRONT, RobotConstants.Indexer.indexerServoCorral);
//            }

            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new ConditionalCommand(
                                    new SequentialCommandGroup(
                                            new TransferBallCommand(Indexer.SlotID.FRONT)
                                            //new WaitCommand(0)
                                    ),
                                    new InstantCommand(),
                                    () -> occupiedList.get(0)
                            ),

                            new ConditionalCommand(
                                    new SequentialCommandGroup(
                                            new TransferBallCommand(Indexer.SlotID.MIDDLE)
                                            //new WaitCommand(50)
                                    ),
                                    new InstantCommand(),
                                    () -> occupiedList.get(1)
                            ),

                            new ConditionalCommand(
                                    new SequentialCommandGroup(
                                            new TransferBallCommand(Indexer.SlotID.REAR)
                                    ),
                                    new InstantCommand(),
                                    () -> occupiedList.get(2)
                            ),

                            new ParallelCommandGroup(
                                    new IndexerArmCommand(Indexer.SlotID.FRONT, RobotHardware.getInstance().indexer.getIndexerServoStowedPosition(Indexer.SlotID.MIDDLE)),
                                    new IndexerArmCommand(Indexer.SlotID.MIDDLE, RobotHardware.getInstance().indexer.getIndexerServoStowedPosition(Indexer.SlotID.REAR)),
                                    new IndexerArmCommand(Indexer.SlotID.REAR, RobotHardware.getInstance().indexer.getIndexerServoStowedPosition(Indexer.SlotID.FRONT))
                            ),

                            new InstantCommand( () -> transferActive = false)
                    )

//                    new ParallelCommandGroup(
//                            new ConditionalCommand(
//                                    new SequentialCommandGroup(
//                                            new IndexerArmCommand(Indexer.SlotID.MIDDLE, RobotHardware.getInstance().indexer.getIndexerServoUpPosition(Indexer.SlotID.MIDDLE)),
//                                            new WaitCommand(160),
//                                            new IndexerArmCommand(Indexer.SlotID.MIDDLE, RobotHardware.getInstance().indexer.getIndexerServoStowedPosition(Indexer.SlotID.MIDDLE))
//                                    ),
//                                    new InstantCommand(),
//                                    () -> occupiedList.get(0)
//                            ),
//
//                            new ConditionalCommand(
//                                    new SequentialCommandGroup(
//                                            new WaitCommand(140),
//                                            new IndexerArmCommand(Indexer.SlotID.REAR, RobotHardware.getInstance().indexer.getIndexerServoUpPosition(Indexer.SlotID.REAR)),
//                                            new WaitCommand(160),
//                                            new IndexerArmCommand(Indexer.SlotID.REAR, RobotHardware.getInstance().indexer.getIndexerServoStowedPosition(Indexer.SlotID.REAR))
//                                    ),
//                                    new InstantCommand(),
//                                    () -> occupiedList.get(1)
//                            ),
//
//                            new ConditionalCommand(
//                                    new SequentialCommandGroup(
//                                            new WaitCommand(460),
//                                            new IndexerArmCommand(Indexer.SlotID.FRONT, RobotHardware.getInstance().indexer.getIndexerServoUpPosition(Indexer.SlotID.FRONT)),
//                                            new WaitCommand(160),
//                                            new IndexerArmCommand(Indexer.SlotID.FRONT, RobotHardware.getInstance().indexer.getIndexerServoStowedPosition(Indexer.SlotID.FRONT))
//                                    ),
//                                    new InstantCommand(),
//                                    () -> occupiedList.get(2)
//                            )
//                    )
            );
        }
    }
}
