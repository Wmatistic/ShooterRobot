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

    public class IndexerSlot {
        private final RevColorSensorV3 sensorA, sensorB;
        public boolean occupied;
        public BallColor color;

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

            if (closeA) {
                BallColor colorA = classifyColor(sensorA);

                occupied = true;
                color = colorA;
            } else if (closeB) {
                BallColor colorB = classifyColor(sensorB);

                occupied = true;
                color = colorB;
            } else {
                occupied = false;
                color = BallColor.NONE;
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
