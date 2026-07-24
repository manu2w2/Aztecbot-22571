package Subsistemas;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class TurretSubsystem_Autonomous extends SubsystemBase {

    private final DcMotorEx turret;
    private static final double POWER = 0.8;     // Aumentado un poco (mejor respuesta)
    private static final int TOLERANCE = 15;     // Tolerancia en ticks

    private double targetPosition = 0.0;

    public TurretSubsystem_Autonomous(HardwareMap hardwareMap) {
        turret = hardwareMap.get(DcMotorEx.class, "TurretMotor");

        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ✅ Corrección importante:
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  // Modo seguro inicial
    }

    /**
     * Establece la posición objetivo de la torreta
     */
    public void setTargetPosition(double position) {
        this.targetPosition = position;

        turret.setTargetPosition((int) Math.round(position));
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);   // Solo ahora cambiamos el modo
        turret.setPower(POWER);
    }

    /**
     * Sobrecarga para compatibilidad con enteros
     */
    public void setTargetPosition(int position) {
        setTargetPosition((double) position);
    }

    /**
     * Verifica si llegó a la posición objetivo
     */
    public boolean isAtTarget() {
        return !turret.isBusy() ||
                Math.abs(turret.getCurrentPosition() - targetPosition) < TOLERANCE;
    }

    public int getCurrentPosition() {
        return turret.getCurrentPosition();
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public void stop() {
        turret.setPower(0);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}