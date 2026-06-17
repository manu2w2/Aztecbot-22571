package Subsistemas;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;


@Configurable
public class subturret extends SubsystemBase {

    private final MotorEx TurretMotor;
    private final GoBildaPinpointDriver pinpoint;

    private final PIDFController TurretController;

    public static double turretKP = 0.015;
    public static double turretKI = 0.0;
    public static double turretKD = 0.000;

    public static double motorEncoderTicksPerRev = 28.0;
    public static double motorGearboxRatio = 15.0;
    public static double turretExternalGearRatio = 4.8;

    public static double turretMaxPower = 0.50;


    public static double turretStartAngle = 90.0;
    public static double turretToleranceTicks = 1.5;
    private boolean enabled = true;

    private double heading;
    private double currentTicks;
    private double currentAngle;
    private double targetAngle;
    private double targetTicks;
    private double errorTicks;


    public static double turretMinAngle = 0.0;
    public static double turretMaxAngle = 180.0;

    private double turretPower;

    public subturret(final HardwareMap hM, final String turretMotorName, final String pinpointName) {
        TurretMotor = new MotorEx(hM, turretMotorName);
        TurretMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        TurretMotor.stopAndResetEncoder();
        TurretMotor.setRunMode(Motor.RunMode.RawPower);

        TurretController = new PIDFController(turretKP, turretKI, turretKD,0);
        pinpoint = hM.get(GoBildaPinpointDriver.class, pinpointName);
        pinpoint.resetPosAndIMU();


    }
    @Override
    public void periodic() {
        pinpoint.update();

        if (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
            stop();
            return;}
        heading = pinpoint.getHeading(UnnormalizedAngleUnit.DEGREES);


        if (enabled){

            currentTicks = TurretMotor.getCurrentPosition();
            currentAngle = turretTicksToDegrees(currentTicks);
            double desiredAngle = turretStartAngle + heading;
            targetAngle = findBestTurretTarget(desiredAngle, currentAngle);
            targetTicks = Math.round(degreesToTurretTicks(targetAngle) * 10.0) / 10.0;
            TurretController.setPIDF(turretKP,turretKI,turretKD,0);
            TurretController.setSetPoint(targetTicks);
            turretPower = TurretController.calculate(currentTicks);
            turretPower = clamp(turretPower, -turretMaxPower, turretMaxPower);
            errorTicks = targetTicks - currentTicks;

            if (Math.abs(errorTicks) <= turretToleranceTicks) {
                turretPower = 0;
            }

            if (currentAngle <= turretMinAngle && turretPower < 0) {
                turretPower = 0;
            }

            if (currentAngle >= turretMaxAngle && turretPower > 0) {
                turretPower = 0;
            }


            TurretMotor.set(turretPower);
        }
        else {stop();}


    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }



    private double getTicksPerTurretRev() {
        return motorEncoderTicksPerRev * motorGearboxRatio * turretExternalGearRatio;
    }

    private double degreesToTurretTicks(double turretAngle) {
        double ticksPerTurretRev = getTicksPerTurretRev();

        return (turretAngle - turretStartAngle) * ticksPerTurretRev / 360.0;
    }

    private double turretTicksToDegrees(double ticks) {
        double ticksPerTurretRev = getTicksPerTurretRev();

        return turretStartAngle + ticks * 360.0 / ticksPerTurretRev;
    }

    private double findBestTurretTarget(double desiredAngle, double currentAngle) {
        double bestAngle = desiredAngle;
        double bestDistance = Double.MAX_VALUE;
        boolean foundValid = false;

        for (int i = -64; i <= 64; i++) {
            double candidate = desiredAngle + 360.0 * i;

            if (candidate >= turretMinAngle && candidate <= turretMaxAngle) {
                double distance = Math.abs(candidate - currentAngle);

                if (distance < bestDistance) {
                    bestDistance = distance;
                    bestAngle = candidate;
                    foundValid = true;
                }
            }
        }

        if (foundValid) {
            return bestAngle;
        }

        return clamp(desiredAngle, turretMinAngle, turretMaxAngle);
    }

    public void enable() {enabled = true;}
    public void disable(){enabled = false; stop();}
    public boolean isEnabled(){return enabled;}
    public void resetHeading() {
        pinpoint.setHeading(0.0, AngleUnit.DEGREES);
        pinpoint.update();
        heading = 0.0;
    }
    public void resetEncoder() {
        TurretMotor.stopAndResetEncoder();
        TurretMotor.setRunMode(Motor.RunMode.RawPower);
        currentTicks = 0;
        currentAngle = turretStartAngle;}
    public void resetAll(){resetEncoder();resetHeading(); stop();}
    public void stop() {
        TurretMotor.stopMotor();
        turretPower = 0;
    }

    public double getHeading() {
        return heading;
    }

    public double getCurrentTicks() {
        return currentTicks;
    }

    public double getCurrentAngle() {
        return currentAngle;
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

