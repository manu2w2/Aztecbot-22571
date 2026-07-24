package org.firstinspires.ftc.teamcode.COMANDOS_AUTONOMOS;

import Subsistemas.TopeSubsystem;
import com.seattlesolvers.solverslib.command.CommandBase;

public class OpenTopeCommand extends CommandBase {

    private final TopeSubsystem tope;

    public OpenTopeCommand(TopeSubsystem tope) {
        this.tope = tope;
        addRequirements(tope);
    }

    @Override
    public void initialize() {
        tope.open();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}