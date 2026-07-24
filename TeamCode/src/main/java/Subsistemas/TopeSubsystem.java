package Subsistemas;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class TopeSubsystem extends SubsystemBase {

    private final ServoEx tope;

    public TopeSubsystem(HardwareMap hardwareMap) {
        tope = new ServoEx(hardwareMap, "ServoTope");
        tope.setCachingTolerance(0.001);
    }

    public void open() {
        tope.set(0.41);
    }

    public void close() {
        tope.set(0.1);
    }
}