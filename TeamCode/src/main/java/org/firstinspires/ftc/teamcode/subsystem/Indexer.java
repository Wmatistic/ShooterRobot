package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class Indexer implements Subsystem {
    public enum BallColor {
        GREEN,
        PURPLE,
        UNKNOWN,
        NONE
    }

    private enum WhichSensor {
        A,
        B,
        NONE
    };

    public class IndexerSlot {
        private final RevColorSensorV3 sensorA, sensorB;
        public boolean occupied;
        public BallColor color;

        private boolean colorLatched = false;
        private WhichSensor latchedSensor = WhichSensor.NONE;

        public IndexerSlot(boolean occupied, BallColor color, RevColorSensorV3 sensorA, RevColorSensorV3 sensorB) {
            this.occupied = occupied;
            this.color = color;
            this.sensorA = sensorA;
            this.sensorB = sensorB;
        }

        public void update() {
            double dA = sensorA.getDistance(DistanceUnit.MM);
            double dB = sensorB.getDistance(DistanceUnit.MM);

            boolean closeA = dA < RobotConstants.Indexer.proximityThreshold;
            boolean closeB = dB < RobotConstants.Indexer.proximityThreshold;

            if (!closeA && !closeB) {
                occupied = false;
                color = BallColor.NONE;
                colorLatched = false;
                latchedSensor = WhichSensor.NONE;
                return;
            }

            // Ball present
            occupied = true;

            WhichSensor seeing;
            if (latchedSensor == WhichSensor.A && closeA) {
                seeing = WhichSensor.A;
            } else if (latchedSensor == WhichSensor.B && closeB) {
                seeing = WhichSensor.B;
            } else {
                if (closeA && closeB) {
                    seeing = (dA <= dB) ? WhichSensor.A : WhichSensor.B;
                } else {
                    seeing = closeA ? WhichSensor.A : WhichSensor.B;
                }
            }

            if (!colorLatched) {
                RevColorSensorV3 s = (seeing == WhichSensor.A) ? sensorA : sensorB;
                color = classifyColor(s);
                colorLatched = true;
                latchedSensor = seeing;
            }
        }

        public boolean getOccupiedStatus() {
            return occupied;
        }

        public BallColor getBallColor() {
            return color;
        }

        private BallColor classifyColor(RevColorSensorV3 s) {
            int r = s.red();
            int g = s.green();
            int b = s.blue();

            // Green ball: green channel dominates
            // Purple ball: red + blue dominate over green
            if (g > r * 1.2 && g > b * 1.2) {
                return BallColor.GREEN;
            } else if ((r + b) > g * 1.4) {
                return BallColor.PURPLE;
            } else {
                return BallColor.UNKNOWN;
            }
        }
    }

    private final RobotHardware robot;

    public enum SlotID {
        FRONT,
        MIDDLE,
        REAR
    }

    private final IndexerSlot[] indexerSlots = new IndexerSlot[3];

    public Indexer() {
        this.robot = RobotHardware.getInstance();

        indexerSlots[0] = new IndexerSlot(
                false,
                BallColor.NONE,
                robot.frontLeftSensor,
                robot.frontRightSensor
        );
        indexerSlots[1] = new IndexerSlot(
                false,
                BallColor.NONE,
                robot.middleLeftSensor,
                robot.middleRightSensor
        );
        indexerSlots[2] = new IndexerSlot(
                false,
                BallColor.NONE,
                robot.rearLeftSensor,
                robot.rearRightSensor
        );
    }

    @Override
    public void periodic() {
        checkIndexerSlots();
    }

    public void moveIndexerSlotServo(SlotID slotID, double position) {
        switch (slotID) {
            case FRONT:
                robot.frontServo.setPosition(position);
                break;
            case MIDDLE:
                robot.middleServo.setPosition(position);
                break;
            case REAR:
                robot.rearServo.setPosition(position);
        }
    }

    public double getIndexerServoStowedPosition(SlotID slotID) {
        switch (slotID) {
            case FRONT:
                return RobotConstants.Indexer.frontIndexerServoStowed;
            case MIDDLE:
                return RobotConstants.Indexer.middleIndexerServoStowed;
            case REAR:
                return RobotConstants.Indexer.rearIndexerServoStowed;
            default:
                return RobotConstants.Indexer.indexerServoStowed;
        }
    }

    public double getIndexerServoUpPosition(SlotID slotID) {
        switch (slotID) {
            case FRONT:
                return RobotConstants.Indexer.frontIndexerServoUp;
            case MIDDLE:
                return RobotConstants.Indexer.middleIndexerServoUp;
            case REAR:
                return RobotConstants.Indexer.rearIndexerServoUp;
            default:
                return RobotConstants.Indexer.indexerServoUp;
        }
    }

    public IndexerSlot getIndexerSlot(SlotID slotID) {
        return indexerSlots[slotID.ordinal()];
    }

    public void checkIndexerSlots() {
        for (IndexerSlot slot : indexerSlots) {
            slot.update();
        }
    }

    public boolean isFull() {
        for (IndexerSlot slot : indexerSlots) {
            if (!slot.getOccupiedStatus()) return false;
        }
        return true;
    }

    public int getBallCount() {
        int count = 0;
        for (IndexerSlot slot : indexerSlots) {
            if (slot.getOccupiedStatus()) count++;
        }
        return count;
    }
}
