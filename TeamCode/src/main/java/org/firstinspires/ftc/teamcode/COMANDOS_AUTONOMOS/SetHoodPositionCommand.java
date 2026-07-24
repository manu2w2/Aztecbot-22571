package org.firstinspires.ftc.teamcode.COMANDOS_AUTONOMOS;

import Subsistemas.HoodSubsystem;
import com.seattlesolvers.solverslib.command.CommandBase;

public class SetHoodPositionCommand extends CommandBase {

    private final HoodSubsystem hood;
    private final double position;

    public SetHoodPositionCommand(HoodSubsystem hood, double position) {
        this.hood = hood;
        this.position = position;
        addRequirements(hood);
    }

    @Override
    public void initialize() {
        hood.setPosition(position);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}