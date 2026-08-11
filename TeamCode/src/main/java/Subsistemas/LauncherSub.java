package Subsistemas;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

@Configurable
public class LauncherSub extends SubsystemBase {

    private static final double LIMELIGHT_MOUNT_ANGLE_DEGREES = 15.0;
    private static final double LIMELIGHT_LENS_HEIGHT_CM = 28.0;
    private static final double APRILTAG_HEIGHT_CM = 75.6;

    private static final double KP = 0.0085;
    private static final double KV = 0.000455;

    private final PController flywheelController = new PController(KP);

    private final MotorGroup flywheelMotors;
    private final MotorEx motor1, motor2;
    private final ServoEx servoHood;
    private final Limelight3A limelight;

    private static final PwmControl.PwmRange HOOD_RANGE = new PwmControl.PwmRange(500, 2500);

    private boolean shooterRunning = false;
    private double motorPower = 0.0;
    private double motorTicksPerSec = 0.0;
    private double goalDistanceCm = 0.0;
    private double TARGET_TICKS_PER_SEC;
    public int TARGET_TAG = 24;

    public LauncherSub(HardwareMap hm, String shooterMotor1, String shooterMotor2, String servoHoodName) {
        motor1 = new MotorEx(hm, shooterMotor1).setCachingTolerance(0.001);
        motor2 = new MotorEx(hm, shooterMotor2).setCachingTolerance(0.001);

        flywheelMotors = new MotorGroup(motor1.setInverted(true), motor2);
        flywheelMotors.setRunMode(Motor.RunMode.RawPower);
        flywheelMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        servoHood = new ServoEx(hm, servoHoodName)
                .setPwm(HOOD_RANGE)
                .setCachingTolerance(0.001);
        servoHood.setInverted(true);

        limelight = hm.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    @Override
    public void periodic() {
        updateGoalDistanceFromVision();

        servoHood.set(hoodAngle(goalDistanceCm) + 0.12);
        TARGET_TICKS_PER_SEC = flywheelSpeed(goalDistanceCm);

        motorTicksPerSec = motor2.getCorrectedVelocity();

        if (!shooterRunning) {
            flywheelMotors.set(0.31);
            return;
        }

        flywheelController.setSetPoint(TARGET_TICKS_PER_SEC);
        double pidOutput = flywheelController.calculate(motorTicksPerSec);
        double feedForward = KV * TARGET_TICKS_PER_SEC;

        motorPower = MathFunctions.clamp(pidOutput + feedForward, -1.0, 1.0);
        flywheelMotors.set(motorPower);
    }

    public void updateGoalDistanceFromVision() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid() || result.getFiducialResults().isEmpty()) {
            return;
        }

        for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
            if (fiducial.getFiducialId() == TARGET_TAG) {
                double ty = fiducial.getTargetYDegrees();
                double angleToGoal = LIMELIGHT_MOUNT_ANGLE_DEGREES + ty;
                double heightDiff = APRILTAG_HEIGHT_CM - LIMELIGHT_LENS_HEIGHT_CM;
                goalDistanceCm = Math.abs(heightDiff / Math.tan(Math.toRadians(angleToGoal)));
                break;
            }
        }
    }

    public static double hoodAngle(double goalDist) {
        double angle = ((-1.26702e-7 * goalDist + 0.000053276) * goalDist - 0.00527341) * goalDist + 0.448478;
        return MathFunctions.clamp(angle, 0.1, 0.95);
    }

    public static double flywheelSpeed(double goalDist) {
        double rpm = ((-0.000111811 * goalDist + 0.0614737) * goalDist - 7.61823) * goalDist + 1278.1689;
        return MathFunctions.clamp(rpm, 300, 1600);
    }

    public void setTargetTag(int tag) { TARGET_TAG = tag; }
    public void startShooter() { shooterRunning = true; flywheelController.reset(); }
    public void stopShooter() { shooterRunning = false; flywheelMotors.set(0.0); }
    public void toggleShooter() { if (shooterRunning) stopShooter(); else startShooter(); }

    public double getTicksPerSec() { return motorTicksPerSec; }
    public double getTicksPerSecError() { return TARGET_TICKS_PER_SEC - motorTicksPerSec; }
    public double getDistance() { return goalDistanceCm; }
    public boolean isShooterRunning() { return shooterRunning; }
}