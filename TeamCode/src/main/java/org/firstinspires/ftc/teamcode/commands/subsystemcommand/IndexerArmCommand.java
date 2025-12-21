package org.firstinspires.ftc.teamcode.commands.subsystemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystem.Indexer;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

public class IndexerArmCommand extends InstantCommand {
    public IndexerArmCommand(Indexer.SlotID slotID, double position) {
        super(
                () -> RobotHardware.getInstance().indexer.moveIndexerSlotServo(slotID, position)
        );
    }
}
