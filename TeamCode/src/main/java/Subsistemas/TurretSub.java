package Subsistemas;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

@Configurable
public class TurretSub extends SubsystemBase {

    private final MotorEx turretMotor;
    private final PController turretController;
    public static double turretKP = 0.015;

    // ================= ENCODER / GEAR RATIO =================
    public static double motorEncoderTicksPerRev = 28.0;
    public static double motorGearboxRatio = 10.43290;
    public static double turretExternalGearRatio = 4.8;

    // Precalculamos el total de Ticks por revolución para no multiplicar en cada loop
    private final double TICKS_PER_TURRET_REV = motorEncoderTicksPerRev * motorGearboxRatio * turretExternalGearRatio;

    public static double turretMaxPower = 0.50;
    public static double turretToleranceTicks = 2.0;
    public static double turretMinAngle = -141;
    public static double turretMaxAngle = 155;

    public static double goalX = 144;
    public static double goalY = 144;
    public static double manualAimOffsetDegrees = 0;

    private boolean enabled = true;
    private double currentTicks = 0.0;
    private double currentAngle = 0;
    private double desiredAngle = 0;
    private double targetAngle = 0;
    private double targetTicks = 0.0;
    private double errorTicks = 0.0;
    private double turretPower = 0.0;
    private static Pose robotPose;

    public TurretSub(HardwareMap hardwareMap, String turretMotorName) {
        turretMotor = new MotorEx(hardwareMap, turretMotorName)
                .setCachingTolerance(0.001);

        turretMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        turretMotor.stopAndResetEncoder();
        turretMotor.setRunMode(Motor.RunMode.RawPower);

        turretController = new PController(turretKP);
    }

    @Override
    public void periodic() {
        if (robotPose == null) return;

        currentTicks = turretMotor.getCurrentPosition();
        currentAngle = ticksToDegrees(currentTicks);

        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double heading = Math.toDegrees(robotPose.getHeading());

        double deltaX = goalX - robotX;
        double deltaY = goalY - robotY;

        desiredAngle = Math.toDegrees(Math.atan2(deltaY, deltaX));
        double relativeTargetAngle = normalizeDegrees(desiredAngle - heading + manualAimOffsetDegrees);

        targetAngle = findBestTurretTarget(-relativeTargetAngle, currentAngle);
        targetTicks = degreesToTicks(targetAngle);

        if (!enabled) {
            stopMotor();
            return;
        }

        turretController.setP(turretKP);
        turretController.setSetPoint(targetTicks);

        turretPower = turretController.calculate(currentTicks);
        turretPower = MathFunctions.clamp(turretPower, -turretMaxPower, turretMaxPower);

        errorTicks = targetTicks - currentTicks;

        if (Math.abs(errorTicks) <= turretToleranceTicks) {
            turretPower = 0.0;
        }

        if (currentAngle <= turretMinAngle && turretPower < 0) {
            turretPower = 0.0;
        }

        if (currentAngle >= turretMaxAngle && turretPower > 0) {
            turretPower = 0.0;
        }

        turretMotor.set(turretPower);
    }

    public void setPose(Pose RobotPose) { robotPose = RobotPose; }
    public void setManualAimOffsetDegrees(double offset) { manualAimOffsetDegrees = offset; }
    public void setGoalX(double GoalX) { goalX = GoalX; }
    public void setGoalY(double GoalY) { goalY = GoalY; }

    private double degreesToTicks(double degrees) {
        return degrees * TICKS_PER_TURRET_REV / 360.0;
    }

    private double ticksToDegrees(double ticks) {
        return ticks * 360.0 / TICKS_PER_TURRET_REV;
    }

    private double findBestTurretTarget(double desiredAngle, double currentAngle) {
        double bestAngle = desiredAngle;
        double bestDistance = Double.MAX_VALUE;
        boolean foundValidTarget = false;

        for (int rotation = -1; rotation <= 1; rotation++) {
            double candidate = desiredAngle + 360.0 * rotation;
            if (candidate < turretMinAngle || candidate > turretMaxAngle) continue;

            double distance = Math.abs(candidate - currentAngle);
            if (distance < bestDistance) {
                bestDistance = distance;
                bestAngle = candidate;
                foundValidTarget = true;
            }
        }

        return foundValidTarget ? bestAngle : MathFunctions.clamp(desiredAngle, turretMinAngle, turretMaxAngle);
    }

    public void enable() { turretController.reset(); enabled = true; }
    public void disable() { enabled = false; turretController.reset(); stopMotor(); }
    public boolean isEnabled() { return enabled; }

    public void stopMotor() {
        turretMotor.stopMotor();
        turretPower = 0.0;
    }

    public double getCurrentAngle() { return currentAngle; }
    public double getTargetAngle() { return targetAngle; }
    public double getAppliedPower() { return turretPower; }
}