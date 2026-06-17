package Subsistemas;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;



@Configurable
public class TurretSub extends SubsystemBase {

    // Hardware
    private final MotorEx turretMotor;
    private final GoBildaPinpointDriver pinpoint;

    // Controlador
    private final PIDFController turretController;

    // PID
    public static double turretKP = 0.015;
    public static double turretKI = 0.0;
    public static double turretKD = 0.0;

    // Relación del encoder y engranajes
    public static double motorEncoderTicksPerRev = 28.0;
    public static double motorGearboxRatio = 15.0;
    public static double turretExternalGearRatio = 4.8;

    // Potencia máxima
    public static double turretMaxPower = 0.50;

    /*
     * Ángulo de la torreta cuando el encoder está en 0.
     * La torreta comienza mirando al frente a 90 grados.
     */
    public static double turretStartAngle = 90.0;

    // Tolerancia del PID
    public static double turretToleranceTicks = 1.5;

    // Límites físicos de la torreta
    public static double turretMinAngle = 0.0;
    public static double turretMaxAngle = 180.0;

    /*
     * Cambia a -1 si la torreta corrige hacia
     * el lado contrario al movimiento del robot.
     */
    public static double headingDirection = 1.0;

    // Estado del subsistema
    private boolean enabled = true;

    // Variables para telemetría
    private double heading = 0.0;
    private double currentTicks = 0.0;
    private double currentAngle = turretStartAngle;

    private double desiredAngle = turretStartAngle;
    private double targetAngle = turretStartAngle;
    private double targetTicks = 0.0;

    private double errorTicks = 0.0;
    private double turretPower = 0.0;

    private final static double goalX = 0;
    private final static double goalY = 0;

    DistanceUnit distanceUnit = DistanceUnit.CM;


    public TurretSub(HardwareMap hardwareMap, String turretMotorName, String pinpointName) {
        turretMotor = new MotorEx(hardwareMap, turretMotorName);

        turretMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        turretMotor.stopAndResetEncoder();

        turretMotor.setRunMode(Motor.RunMode.RawPower);

        turretController = new PIDFController(turretKP, turretKI, turretKD, 0);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, pinpointName);

        pinpoint.resetPosAndIMU();
    }


    @Override
    public void periodic()  {

        // Actualizar Pinpoint una vez por ciclo
        pinpoint.update();

        // Leer el encoder incluso si el Pinpoint aún no está listo
        currentTicks = turretMotor.getCurrentPosition();
        currentAngle = turretTicksToDegrees(currentTicks);

        double robotY = pinpoint.getPosY(distanceUnit);
        double robotX = pinpoint.getPosX(distanceUnit);

        double theta = findTheta(robotX, robotY);


        /*
         * No mover la torreta mientras el Pinpoint
         * se está inicializando o tiene algún error.
         */
        if (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
            stopMotor();
            return;
        }

        heading = pinpoint.getHeading(UnnormalizedAngleUnit.DEGREES);

        if (!enabled) {
            stopMotor();
            return;
        }

        /*
         * La posición deseada cambia con el heading
         * para mantener la orientación de la torreta.
         */
        desiredAngle = turretStartAngle
                + heading + theta * headingDirection;

        targetAngle = findBestTurretTarget(
                desiredAngle,
                currentAngle
        );

        targetTicks = degreesToTurretTicks(targetAngle);

        // Redondear a una décima de tick
        targetTicks = Math.round(targetTicks * 10.0) / 10.0;

        turretController.setPIDF(
                turretKP,
                turretKI,
                turretKD,
                0
        );

        turretController.setSetPoint(targetTicks);

        turretPower = turretController.calculate(
                currentTicks
        );

        turretPower = clamp(
                turretPower,
                -turretMaxPower,
                turretMaxPower
        );

        errorTicks = targetTicks - currentTicks;

        // Detenerse dentro de la tolerancia
        if (Math.abs(errorTicks)
                <= turretToleranceTicks) {

            turretPower = 0.0;
        }

        // Límite mínimo de software
        if (currentAngle <= turretMinAngle
                && turretPower < 0.0) {

            turretPower = 0.0;
        }

        // Límite máximo de software
        if (currentAngle >= turretMaxAngle
                && turretPower > 0.0) {

            turretPower = 0.0;
        }

        turretMotor.set(turretPower);
    }


    private double clamp(
            double value,
            double minimum,
            double maximum
    ) {
        return Math.max(
                minimum,
                Math.min(maximum, value)
        );
    }


    public double getTicksPerTurretRev() {
        return motorEncoderTicksPerRev
                * motorGearboxRatio
                * turretExternalGearRatio;
    }


    private double degreesToTurretTicks(
            double turretAngle
    ) {
        return (turretAngle - turretStartAngle)
                * getTicksPerTurretRev()
                / 360.0;
    }


    private double turretTicksToDegrees(
            double ticks
    ) {
        return turretStartAngle
                + ticks
                * 360.0
                / getTicksPerTurretRev();
    }

    private double findTheta(double robotX, double robotY) {
        double deltaX = TurretSub.goalX - robotX;
        double deltaY = TurretSub.goalY - robotY;
        return Math.toDegrees(Math.atan2(deltaX, deltaY));
    }
    private double findBestTurretTarget(double desiredAngle, double currentAngle) {
        double bestAngle = desiredAngle;
        double bestDistance = Double.MAX_VALUE;

        boolean foundValidTarget = false;

        /*
         * Busca una representación equivalente del ángulo
         * que se encuentre dentro de los límites.
         */
        for (int rotation = -64; rotation <= 64; rotation++) {

            double candidate =
                    desiredAngle + 360.0 * rotation;

            boolean insideLimits =
                    candidate >= turretMinAngle
                            && candidate <= turretMaxAngle;

            if (!insideLimits) {
                continue;
            }

            double distance = Math.abs(
                    candidate - currentAngle
            );

            if (distance < bestDistance) {
                bestDistance = distance;
                bestAngle = candidate;
                foundValidTarget = true;
            }
        }

        if (foundValidTarget) {
            return bestAngle;
        }

        /*
         * Si no existe una posición equivalente dentro
         * de los límites, se usa el límite más cercano.
         */
        return clamp(
                desiredAngle,
                turretMinAngle,
                turretMaxAngle
        );
    }


    public void enable() {
        turretController.reset();
        enabled = true;
    }


    public void disable() {
        enabled = false;
        turretController.reset();
        stopMotor();
    }


    public boolean isEnabled() {
        return enabled;
    }


    public void toggleEnabled() {
        if (enabled) {
            disable();
        } else {
            enable();
        }
    }


    public void resetHeading() {
        stopMotor();

        pinpoint.setHeading(
                0.0,
                AngleUnit.DEGREES
        );

        pinpoint.update();

        heading = 0.0;
        desiredAngle = turretStartAngle;
        targetAngle = turretStartAngle;
        targetTicks = 0.0;

        turretController.reset();
    }


    public void resetEncoder() {
        stopMotor();

        turretMotor.stopAndResetEncoder();

        turretMotor.setRunMode(
                Motor.RunMode.RawPower
        );

        currentTicks = 0.0;
        currentAngle = turretStartAngle;

        targetTicks = 0.0;
        targetAngle = turretStartAngle;
        errorTicks = 0.0;

        turretController.reset();
    }


    public void resetAll() {
        stopMotor();

        resetEncoder();
        resetHeading();

        turretController.reset();
    }


    public void stopMotor() {
        turretMotor.stopMotor();
        turretPower = 0.0;
    }


    // Getters para telemetría

    public double getHeading() {
        return heading;
    }


    public double getCurrentTicks() {
        return currentTicks;
    }


    public double getCurrentAngle() {
        return currentAngle;
    }


    public double getDesiredAngle() {
        return desiredAngle;
    }


    public double getTargetTicks() {
        return targetTicks;
    }


    public double getTargetAngle() {
        return targetAngle;
    }


    public double getErrorTicks() {
        return errorTicks;
    }


    public double getAppliedPower() {
        return turretPower;
    }


    public GoBildaPinpointDriver.DeviceStatus getPinpointStatus() {
        return pinpoint.getDeviceStatus();
    }


    public double getPinpointFrequency() {
        return pinpoint.getFrequency();
    }


    public double getHeadingVelocity() {
        return pinpoint.getHeadingVelocity(
                UnnormalizedAngleUnit.DEGREES
        );
    }
}