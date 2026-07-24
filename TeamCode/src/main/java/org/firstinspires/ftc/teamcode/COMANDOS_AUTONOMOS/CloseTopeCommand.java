package org.firstinspires.ftc.teamcode.COMANDOS_AUTONOMOS;

import Subsistemas.TopeSubsystem;
import com.seattlesolvers.solverslib.command.CommandBase;

public class CloseTopeCommand extends CommandBase {

    private final TopeSubsystem tope;

    public CloseTopeCommand(TopeSubsystem tope) {
        this.tope = tope;
        addRequirements(tope);
    }

    @Override
    public void initialize() {
        tope.close();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}