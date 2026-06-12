package utilidades;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class imuEx {

    private final IMU imu;

    private double lastRawHeading = 0.0;
    private double continuousHeading = 0.0;

    public imuEx(HardwareMap hardwareMap, String imuName) {
        imu = hardwareMap.get(IMU.class, imuName);
    }

    public void init() {
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;

        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
        reset();
    }

    public void reset() {
        imu.resetYaw();
        lastRawHeading = getRawHeading();
        continuousHeading = 0.0;
    }

    public double getRawHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public double getHeading() {
        return getRawHeading();
    }

    public double getContinuousHeading() {
        double rawHeading = getRawHeading();

        double delta = angleDelta(lastRawHeading, rawHeading);

        continuousHeading += delta;
        lastRawHeading = rawHeading;

        return continuousHeading;
    }

    private double angleDelta(double current, double target) {
        double delta = target - current;

        while (delta > 180.0) {
            delta -= 360.0;
        }

        while (delta < -180.0) {
            delta += 360.0;
        }

        return delta;
    }

    public IMU getInternalIMU() {
        return imu;
    }
}