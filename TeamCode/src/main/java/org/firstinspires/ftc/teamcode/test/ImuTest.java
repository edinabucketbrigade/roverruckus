package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/**
 * Created by phinn on 1/12/2019.
 */
@Autonomous(name="ImuTest", group="Linear Opmode")
public class ImuTest extends LinearOpMode {

        private double RIGHT = 1.0;
        private double LEFT = -1.0;
        private double FORWARD = -1.0;
        private double BACKWARD = 1.0;
        private double FAST = 1.0;
        private double SLOW = 0.5;

        // Declare OpMode members.
        private ElapsedTime runtime = new ElapsedTime();
        private DcMotor leftDrive = null;
        private DcMotor rightDrive = null;
        private DcMotor strafeWheel = null;
        private DcMotor upDown = null;
        private CRServo markerServo = null;

        private BNO055IMU imu;
        private VuforiaLocalizer vuforia;
        private VuforiaLocalizer.Parameters parameters;

        // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
        // We will define some constants and conversions here
        private static final float mmPerInch = 25.4f;
        private static final float mmFTCFieldWidth = (12 * 6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
        private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

        // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
        // Valid choices are:  BACK or FRONT
        private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

        private OpenGLMatrix lastLocation = null;
        private boolean targetVisible = false;

        static {
            System.loadLibrary("opencv_java3");
        }

        @Override
        public void runOpMode() throws InterruptedException {





            telemetry.addData("Status", "Initialized");
            waitForStart();
            InitGyro();



        }

     

        private void InitGyro() {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

            parameters.mode = BNO055IMU.SensorMode.IMU;
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled = false;

            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
            // and named "imu".
            imu = hardwareMap.get(BNO055IMU.class, "imu");

            imu.initialize(parameters);

            telemetry.addData("Mode", "calibrating...");
            telemetry.update();

            // make sure the imu gyro is calibrated before continuing.
            while (!isStopRequested() && !imu.isGyroCalibrated()) {
                sleep(50);
                idle();
            }
        }
}

