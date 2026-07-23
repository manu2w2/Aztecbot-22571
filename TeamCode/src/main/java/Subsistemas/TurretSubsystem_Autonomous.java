package Subsistemas;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class TurretSubsystem_Autonomous extends SubsystemBase {
    private final DcMotorEx turret;
    private static final double POWER = 0.5;
    private static final int TOLERANCE = 10; // ticks

    private double targetPosition = 0.0;   // ← Ahora guardamos como double

    public TurretSubsystem_Autonomous(HardwareMap hardwareMap) {
        turret = hardwareMap.get(DcMotorEx.class, "TurretMotor");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Nueva versión que acepta double (recomendada)
     */
    public void setTargetPosition(double position) {
        this.targetPosition = position;
        turret.setTargetPosition((int) Math.round(position));  // Redondea al tick más cercano
        turret.setPower(POWER);
    }

    /**
     * Mantengo el método original por compatibilidad
     */
    public void setTargetPosition(int position) {
        setTargetPosition((double) position);   // Reutiliza el nuevo método
    }

    public boolean isAtTarget() {
        return !turret.isBusy() ||
                Math.abs(turret.getCurrentPosition() - targetPosition) < TOLERANCE;
    }

    public int getCurrentPosition() {
        return turret.getCurrentPosition();
    }

    public void stop() {
        turret.setPower(0);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public double getTargetPosition() {
        return targetPosition;
    }
}