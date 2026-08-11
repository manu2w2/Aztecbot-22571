package org.firstinspires.ftc.teamcode.opModes.teleOps;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

import Subsistemas.LauncherSubA;
import Subsistemas.TurretSub;

@Configurable
@TeleOp(name = "TELEOP_azul_F_Optimized")
public class TeleOp_azulF extends CommandOpMode {

    private Follower follower;

    // Transferencia
    private DcMotorEx transferMotor;
    // Drivetrain
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    // Servos
    private ServoEx servoTope, Light;

    // Subsistemas
    private LauncherSubA launcher;
    private TurretSub turret;

    private boolean reversetransfer = false;
    private boolean transferRunning = false;
    private boolean shooting = false;
    private double angle = 0;

    // ================= ESTADOS =================
    public static double servoMin = 0.1;
    public static double servoMax = 0.41;
    public static double intakeVel = 1500;

    public double hoodAngle = 0;
    public static double initX = 32.856;
    public static double initY = 13.518;

    public static Pose startingPose = new Pose(initX, initY, Math.toRadians(180));

    private List<LynxModule> allHubs;

    @Override
    public void initialize() {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        follower.startTeleopDrive();

        initializeTurret();
        initializeShooter();
        initializeTransfer();
        initializeDrive();
        initializeServo();

        turret.setGoalX(0);
        turret.setManualAimOffsetDegrees(0);

        register(turret, launcher);

        new Trigger(() -> gamepad2.aWasPressed())
                .whenActive(new InstantCommand(() -> launcher.toggleShooter()));

        new Trigger(() -> gamepad2.yWasPressed())
                .whenActive(new InstantCommand(() -> {
                    transferRunning = !transferRunning;
                    reversetransfer = false;
                }));

        new Trigger(() -> gamepad2.dpadUpWasPressed())
                .whenActive(new InstantCommand(() -> {
                    reversetransfer = !reversetransfer;
                    transferRunning = false;
                }));

        new Trigger(() -> gamepad1.yWasPressed()).whenActive(new InstantCommand(() -> angle += 7.0));
        new Trigger(() -> gamepad1.aWasPressed()).whenActive(new InstantCommand(() -> angle -= 7.0));
        new Trigger(() -> gamepad1.xWasPressed()).whenActive(new InstantCommand(() -> angle += 2.0));
        new Trigger(() -> gamepad1.bWasPressed()).whenActive(new InstantCommand(() -> angle -= 2.0));
    }

    private void initializeTurret() {
        turret = new TurretSub(hardwareMap, "TurretMotor");
    }

    private void initializeShooter() {
        launcher = new LauncherSubA(hardwareMap, "shooter", "shooter2", "hood");
        launcher.setTargetTag(20);
    }

    private void initializeTransfer() {
        transferMotor = hardwareMap.get(DcMotorEx.class, "Transfer");
        transferMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        transferMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void initializeDrive() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void initializeServo() {
        servoTope = new ServoEx(hardwareMap, "ServoTope").setCachingTolerance(0.001);
        Light = new ServoEx(hardwareMap, "light").setCachingTolerance(0.01);
    }

    @Override
    public void run() {
        follower.update();
        turret.setPose(follower.getPose());
        turret.setManualAimOffsetDegrees(angle);

        driveControl();
        servoControl();
        transferControl();

        addTelemetry();
    }

    private void driveControl() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.05;
        double rotation = gamepad1.right_stick_x;

        double frontLeftPower = y + x + rotation;
        double frontRightPower = y - x - rotation;
        double backLeftPower = y - x + rotation;
        double backRightPower = y + x - rotation;

        double maximum = Math.max(1.0, Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower), Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)))));

        frontLeft.setPower(frontLeftPower / maximum);
        frontRight.setPower(frontRightPower / maximum);
        backLeft.setPower(backLeftPower / maximum);
        backRight.setPower(backRightPower / maximum);
    }

    private void servoControl() {
        double error = Math.abs(launcher.getTicksPerSecError());

        if (gamepad2.right_bumper && error <= 30) {
            servoTope.set(servoMax);
            shooting = true;
        } else {
            servoTope.set(servoMin);
            shooting = false;
        }

        boolean shooterRunning = launcher.isShooterRunning();

        if (!shooterRunning && !transferRunning) {
            Light.set(0.61);
        } else if (!shooterRunning) {
            Light.set(0.28);
        } else if (!transferRunning) {
            Light.set(0.39);
        } else {
            Light.set(0.50);
        }
    }

    private void transferControl() {
        intakeVel = (launcher.getDistance() >= 230) ? 1050 : 1500;

        if (shooting) {
            transferMotor.setVelocity(intakeVel);
        } else if (transferRunning) {
            transferMotor.setPower(0.8);
        } else if (reversetransfer) {
            transferMotor.setPower(-0.5);
        } else {
            transferMotor.setPower(0.0);
        }
    }

    private void addTelemetry() {
        telemetry.addData("Motor Velocity", launcher.getTicksPerSec());
        telemetry.addData("Motor Velocity Error", launcher.getTicksPerSecError());
        telemetry.addData("Goal Distance", launcher.getDistance());
        telemetry.addData("Turret Angle", turret.getCurrentAngle());
        telemetry.addData("Target Angle", turret.getTargetAngle());
        telemetry.addData("Hood Angle", hoodAngle);
        telemetry.update();
    }

    @Override
    public void reset() {
        if (turret != null) turret.disable();
        if (transferMotor != null) transferMotor.setPower(0.0);
        if (launcher != null) launcher.stopShooter();
        if (frontLeft != null) frontLeft.setPower(0.0);
        if (frontRight != null) frontRight.setPower(0.0);
        if (backLeft != null) backLeft.setPower(0.0);
        if (backRight != null) backRight.setPower(0.0);

        transferRunning = false;
        super.reset();
    }
}