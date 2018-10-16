package com.edinaftcrobotics.navigation;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.ReentrantLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

public class RevImu extends Thread {
    private BNO055IMU _imu;
    private Orientation _lastAngles = new Orientation();
    private double _globalAngle;
    private Telemetry _telemetry;
    private ReentrantLock _lock = new ReentrantLock();

    public RevImu(HardwareMap hardwareMap, Telemetry telemetry)
    {
        _telemetry = telemetry;
        try {
            initGyro(hardwareMap);
        } catch (Exception ex) {
        }
    }

    public void run() {
        while (true) {
            try {
                sleep(100);
                _lock.tryLock(10, TimeUnit.MILLISECONDS);
                pollForAngle();
            } catch (Exception ex) {

            } finally {
                _lock.unlock();
            }
        }
    }

    private void initGyro (HardwareMap hardwareMap) throws InterruptedException{
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        _imu = hardwareMap.get(BNO055IMU.class, "imu");

        _imu.initialize(parameters);

        _telemetry.addData("Mode", "calibrating...");
        _telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!_imu.isGyroCalibrated()) {
            sleep(50);
        }

        resetAngle();
    }

    private void resetAngle ()
    {
        try {
            _lock.tryLock(10, TimeUnit.MILLISECONDS);
            _lastAngles = _imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            _globalAngle = 0;
        } catch (Exception ex) {

        } finally {
            _lock.unlock();
        }
    }

    private void pollForAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = _imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - _lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        _globalAngle += deltaAngle;

        _lastAngles = angles;
    }

    public double getAngle()
    {
        double currentAngle = 0;

        try {
            _lock.tryLock(10, TimeUnit.MILLISECONDS);
            currentAngle = _globalAngle;
        } catch (Exception ex) {

        } finally {
            _lock.unlock();
        }

        return currentAngle;
    }
}
