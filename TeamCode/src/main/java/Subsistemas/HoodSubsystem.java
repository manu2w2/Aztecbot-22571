package Subsistemas;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class HoodSubsystem extends SubsystemBase {

    private final ServoEx hood;

    public HoodSubsystem(HardwareMap hardwareMap) {
        hood = new ServoEx(hardwareMap, "hood");
        hood.setCachingTolerance(0.001);
    }

    public void setPosition(double position) {
        hood.set(position);
    }
}