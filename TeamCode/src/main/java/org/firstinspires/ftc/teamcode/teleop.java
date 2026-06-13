package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import utilidades.imuEx;



@Config
@TeleOp(name = "TELEOPBETA")
public class teleop extends OpMode {



    DcMotorEx flywheelMotor, flywheelMotor2, Transfer;
    PIDFController shooterController;
    PIDFController TurretController;
    imuEx imu;
    MotorEx TurretMotor;
    ServoEx ServoTope;
    boolean shooterRunning = false;
    boolean transferRunning = false;


    DcMotorEx frontLeft, frontRight, backLeft, backRight;

    public static double targetVel = 750;
    public static double kP = 0.05;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kV = 0.000525;

    public static double turretKP = 0.015;
    public static double turretKI = 0.0;
    public static double turretKD = 0.000;

    public static double motorEncoderTicksPerRev = 28.0;
    public static double motorGearboxRatio = 15.0;
    public static double turretExternalGearRatio = 4.8;

    public static double turretMaxPower = 0.50;


    public static double turretStartAngle = 90.0;
    public static double turretToleranceTicks = 1.5;

    // Límites de software.
// Ajusta estos valores a lo que realmente aguante tu torreta.
    public static double turretMinAngle = 0.0;
    public static double turretMaxAngle = 180.0;


    @Override
    public void init() {
        flywheelMotor  = hardwareMap.get(DcMotorEx.class, "shooter");
        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        Transfer       = hardwareMap.get(DcMotorEx.class, "Transfer");
        ServoTope = new ServoEx(hardwareMap, "ServoTope");


        TurretMotor = new MotorEx(hardwareMap, "TurretMotor");
        TurretMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        TurretMotor.stopAndResetEncoder();
        TurretMotor.setRunMode(Motor.RunMode.RawPower);
        TurretController = new PIDFController(turretKP, turretKI, turretKD, 0);

        imu = new imuEx(hardwareMap, "imu");
        imu.init();


// por cada 4.8 vueltas del engrane del motor la torreta da una vuelta

        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        Transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft  = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight  = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterController = new PIDFController(kP, kI, kD, 0);
    }

    @Override
    public void loop() {
        double heading = imu.getContinuousHeading();

        double currentTurretTicks = TurretMotor.getCurrentPosition();
        double currentTurretAngle = turretTicksToDegrees(currentTurretTicks);
        double desiredTurretAngle = turretStartAngle + heading;
        double targetTurretAngle = findBestTurretTarget(desiredTurretAngle, currentTurretAngle);
        double targetTurretTicks = Math.round(degreesToTurretTicks(targetTurretAngle) * 10.0) / 10.0;
        TurretController.setPIDF(turretKP,turretKI,turretKD,0);
        TurretController.setSetPoint(targetTurretTicks);
        double turretPower = TurretController.calculate(currentTurretTicks);
        turretPower = clamp(turretPower, -turretMaxPower, turretMaxPower);
        double turretErrorTicks = targetTurretTicks - currentTurretTicks;

        if (Math.abs(turretErrorTicks) <= turretToleranceTicks) {
            turretPower = 0;
        }

        if (currentTurretAngle <= turretMinAngle && turretPower < 0) {
            turretPower = 0;
        }

        if (currentTurretAngle >= turretMaxAngle && turretPower > 0) {
            turretPower = 0;
        }


        TurretMotor.set(turretPower); 

        telemetry.addData("Robot heading", heading);
        telemetry.addData("Current turret angle", currentTurretAngle);
        telemetry.addData("Desired turret angle", desiredTurretAngle);
        telemetry.addData("Target turret angle", targetTurretAngle);
        telemetry.addData("Turret power", turretPower);
        telemetry.addData("Current turret ticks", currentTurretTicks);
        telemetry.addData("Target turret ticks", targetTurretTicks);
        telemetry.addData("Turret error ticks", targetTurretTicks - currentTurretTicks);
        telemetry.addData("Ticks per turret rev", getTicksPerTurretRev());
        telemetry.addData("Solvers ticks", TurretMotor.getCurrentPosition());
        telemetry.addData("SDK ticks", TurretMotor.motorEx.getCurrentPosition());

        double y  = -gamepad1.left_stick_y;
        double x  =  gamepad1.left_stick_x;
        double rx =  gamepad1.right_stick_x;

        double fl = y + x + rx;
        double fr = y - x - rx;
        double bl = y - x + rx;
        double br = y + x - rx;

        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));

        frontLeft.setPower(fl / max);
        frontRight.setPower(fr / max);
        backLeft.setPower(bl / max);
        backRight.setPower(br / max);


//
        if (gamepad2.left_bumper) {
            ServoTope.set(0.05);
        } else {
            ServoTope.set(0.5
            );
        }

        if (gamepad2.aWasPressed()) shooterRunning = !shooterRunning;



        if (shooterRunning) {
            double currentVelocity = (flywheelMotor.getVelocity() + flywheelMotor2.getVelocity()) / 2;
            shooterController.setPIDF(kP, kI, kD, 0);
            shooterController.setSetPoint(targetVel);
            telemetry.addData("velocidad: ", currentVelocity);

            double power = (kV * targetVel) + shooterController.calculate(currentVelocity);
            flywheelMotor.setPower(power);
            flywheelMotor2.setPower(power);
        } else {
            flywheelMotor.setPower(0);
            flywheelMotor2.setPower(0);
        }


        if (gamepad2.yWasPressed()) transferRunning = !transferRunning;
        Transfer.setPower(transferRunning ? 1 : 0);

        telemetry.update();
    }

    public double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }



    public double getTicksPerTurretRev() {
        return motorEncoderTicksPerRev * motorGearboxRatio * turretExternalGearRatio;
    }

    public double degreesToTurretTicks(double turretAngle) {
        double ticksPerTurretRev = getTicksPerTurretRev();

        return (turretAngle - turretStartAngle) * ticksPerTurretRev / 360.0;
    }

    public double turretTicksToDegrees(double ticks) {
        double ticksPerTurretRev = getTicksPerTurretRev();

        return turretStartAngle + ticks * 360.0 / ticksPerTurretRev;
    }

    public double findBestTurretTarget(double desiredAngle, double currentAngle) {
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


}