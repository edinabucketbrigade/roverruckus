/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.edinaftcrobotics.vision.camera.BackPhoneCamera;
import com.edinaftcrobotics.vision.camera.Camera;
import com.edinaftcrobotics.vision.tracker.roverruckus.GoldMineralTracker;
import com.edinaftcrobotics.vision.tracker.roverruckus.PictureTracker;
import com.edinaftcrobotics.vision.utils.Triple;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="BaseAutonomous", group="Linear Opmode")
public class BaseAutonomous extends LinearOpMode {

    // basic navigation constants
    public StartPosition startPosition;
    public GoldPosition goldPosition;
    private double RIGHT = 1.0;
    private double LEFT = -1.0;
    private double FORWARD = -1.0;
    private double BACKWARD = 1.0;
    private double FAST = 1.0;
    private double SLOW = 0.5;
    boolean MINERAL_ALIGNED = false;
    boolean SAMPLE_MINERALS = true;

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
    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

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


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "leftWheel");
        rightDrive = hardwareMap.get(DcMotor.class, "rightWheel");
        strafeWheel = hardwareMap.get(DcMotor.class, "strafeWheel");
        upDown = hardwareMap.get(DcMotor.class, "upDown");
        markerServo = hardwareMap.get(CRServo.class, "markerServo");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        upDown.setDirection(DcMotor.Direction.REVERSE);
        markerServo.setDirection(CRServo.Direction.FORWARD);

        while (!opModeIsActive()&&!isStopRequested()) {
            telemetry.addData("Status", "Waiting in Init");
            telemetry.update(); }

        telemetry.addData("Status", "Initialized");
        waitForStart();

        //Lower Robot
        telemetry.addData("Auto step:", "Lower Robot");
        upDown.setPower(0.65);
        sleep(3000);
        upDown.setPower(0.0);
        sleep(250);

        //Detach from bracket
        telemetry.addData("Auto step:", "Detach from bracket");
        strafeWheel.setPower(RIGHT*SLOW);
        sleep(800);
        strafeWheel.setPower(0.0);

        //Drive to minerals
        telemetry.addData("Auto step:", "Drive to minerals");
        driveForwardSlow(1200);

        telemetry.addData("Auto step:", "Get into the middle");
        strafeWheel.setPower(LEFT*SLOW);
        sleep(800);
        strafeWheel.setPower(0.0);

        if (SAMPLE_MINERALS) {
            //Sample minerals
            telemetry.addData("Auto step:", "Sample minerals");
            sampleMinerals();
        }

        //Knock gold off
        if (MINERAL_ALIGNED) {
            telemetry.addData("Auto step:", "Knock off gold");
            driveForward(350);
            telemetry.update();

            //Drive to Depot
            telemetry.addData("Auto step:", "Drive to depot");
            driveToDepot(goldPosition);
            telemetry.update();

            //Drop the marker
            telemetry.addData("Auto step:", "Drop marker");
            dropMarker();
            telemetry.update();
        } else {
            telemetry.addData("DONE", "Nothing left to do.");
            telemetry.update();
        }

    }

    private void sampleMinerals() {

        Camera camera = new BackPhoneCamera();
        camera.activate();
        GoldMineralTracker mineralTracker = new GoldMineralTracker(camera);


        int ALIGNED_VALUE = 330;

        int NOT_FOUND_COUNT = 0;
        double search_direction = RIGHT;
        int NUMBER_OF_TIMES_SEARCHED = 0;

        while (MINERAL_ALIGNED == false || NUMBER_OF_TIMES_SEARCHED >= 4) {
            try {
                if (mineralTracker.getGoldMineralLocation()) {
                    telemetry.addData("Aligned: ", mineralTracker.aligned());
                    MINERAL_ALIGNED = mineralTracker.aligned();

                    telemetry.addData("Location: ", "%f %f", mineralTracker.getXPosition(), mineralTracker.getYPosition());
                    if (MINERAL_ALIGNED == false) {
                        if(mineralTracker.getXPosition() > ALIGNED_VALUE) {
                            strafeWheel.setPower(RIGHT*SLOW);
                            sleep(100);
                            strafeWheel.setPower(0);

                        } else {
                            strafeWheel.setPower(LEFT*SLOW);
                            sleep(100);
                            strafeWheel.setPower(0);
                        }
                    }

                    switch (NOT_FOUND_COUNT) {
                        case 0:
                            goldPosition = GoldPosition.MIDDLE;
                            break;
                        case 1:
                        case 2:
                            goldPosition = GoldPosition.RIGHT;
                            break;
                        case -1:
                        case -2:
                            goldPosition = GoldPosition.LEFT;
                            break;
                        default:
                            goldPosition = GoldPosition.MIDDLE;
                    }


                    telemetry.addData("Gold Position", goldPosition);
                    telemetry.update();

                } else {
                    if (Math.abs(NOT_FOUND_COUNT) >= 2) {
                        search_direction = search_direction * -1.0;
                        NUMBER_OF_TIMES_SEARCHED++;
                    }

                    telemetry.addData("Object Not Found", "");
                    strafeWheel.setPower(search_direction*SLOW);
                    sleep(850);
                    strafeWheel.setPower(0);
                    NOT_FOUND_COUNT += 1 * (int)search_direction;

                }

                sleep(100);
                telemetry.update();
            } catch (InterruptedException e) {
                telemetry.addData("interrupted for some reason...", "");
                telemetry.update();
            }
        }

    }

    public void driveToDepot(GoldPosition goldPosition) {

        if (startPosition == StartPosition.BLUE_CRATER || startPosition == StartPosition.RED_CRATER) {

            //back up so you don't hit the minerals
            telemetry.addData("Auto step:", "Back up");
            driveBackward(350);
            telemetry.update();

            if (goldPosition == GoldPosition.MIDDLE) {
                // rotate 90
                rotateRobot(90);
                driveForward(1400);
                rotateRobot(30);
                driveForward(1400);
            }

            if (goldPosition == GoldPosition.LEFT) {  //LEFT
                // rotate 90
                rotateRobot(85);
                driveForward(1200);
                rotateRobot(20);
                driveForward(1400);
            }

            if (goldPosition == GoldPosition.RIGHT) { //RIGHT ACTUALLY
                // rotate 90
                rotateRobot(85);
                driveForward(1850);
                rotateRobot(37);
                driveForward(1500);
            }
        }

        if (startPosition == StartPosition.BLUE_DEPOT || startPosition == StartPosition.RED_DEPOT) {

            if (goldPosition == GoldPosition.MIDDLE) {
                driveForward(600);
                rotateRobot(25);
                driveForward(200);
            }

            if (goldPosition == GoldPosition.RIGHT) {
                driveForward(500);
                rotateRobot(45);
                driveForward(200);
            }

            if (goldPosition == GoldPosition.LEFT) {
                driveForward(450);

            }
        }


    }

    public void driveForward(int milliseconds) {
        leftDrive.setPower(FORWARD * FAST);
        rightDrive.setPower(FORWARD * FAST);
        sleep(milliseconds);
        leftDrive.setPower(0.0);
        rightDrive.setPower(0.0);
    }

    public void driveForwardSlow(int milliseconds) {
        leftDrive.setPower(FORWARD * SLOW);
        rightDrive.setPower(FORWARD * SLOW);
        sleep(milliseconds);
        leftDrive.setPower(0.0);
        rightDrive.setPower(0.0);
    }

    public void driveBackward(int milliseconds) {
        leftDrive.setPower(BACKWARD * FAST);
        rightDrive.setPower(BACKWARD * FAST);
        sleep(milliseconds);
        leftDrive.setPower(0.0);
        rightDrive.setPower(0.0);
    }


    private void dropMarker() {
        //Drop Marker
        telemetry.addData("Auto step:", "Drop Marker");
        markerServo.setPower(0.5);
        sleep(500);
        markerServo.setPower(0.0);
        markerServo.setPower(-0.5);
        sleep(500);
        markerServo.setPower(0.0);
    }

    private void spinAndLook() {
//        ElapsedTime stopwatch = new ElapsedTime();
//        camera.deactivate();
//
//
//        camera = new BackPhoneCamera();
//        camera.activate();
//
//        sleep(2000);
//
//        PictureTracker pictureTracker = new PictureTracker(camera,0,0,0);
//
//        pictureTracker.startTracking();
//
//
//        stopwatch.reset();
//
//        String desiredPictureName = "Blue-Rover";
//
//        /** Start tracking the data sets we care about. */
//
//
//
//        while (opModeIsActive()) {
//
//            Triple location = pictureTracker.getTrackableObject(telemetry);
//
//            if (location != null) {
//                // it found something
//                telemetry.addData("OBJECT FOUND: ", "%f %f %f", location.Orientation.firstAngle, location.Orientation.secondAngle, location.Orientation.thirdAngle);
//
//                // is it the picture we want?
//                if (desiredPictureName == location.PictureName) {
//
//                    // hooray drive straight towards it
//                    telemetry.addData("DESIRED OBJECT FOUND: ", "%f %f %f", location.Orientation.firstAngle, location.Orientation.secondAngle, location.Orientation.thirdAngle);
//                    telemetry.addData("POINTS", "X%f Y%f Z%f ", location.Point.x, location.Point.y, location.Point.z);
//
//                    if (location.Point.z < 6.0 ) {
//                        rotateRobot(10);
//                    } else if (location.Point.z > 6.4) {
//                        rotateRobot(-10);
//                    } else {
//                        leftDrive.setPower(FORWARD * FAST);
//                        rightDrive.setPower(FORWARD * FAST);
//                        sleep(1000);
//                        leftDrive.setPower(0);
//                        rightDrive.setPower(0);
//
//                        dropMarker();
//                    }
//                }
//
//            }
//
//            // spin some more
//            telemetry.addData("Spinning Nothing found", null);
//            rotateRobot(1);
//            sleep(250);
//
//
//            telemetry.update();
//
//        }
//
//        camera.deactivate();
    }

    private void InitCamera () {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        this.parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "Af0zbuj/////AAAAmXrKvO2DQ02rqj1ToEd4GiNXW/eJaJfXEGRXIRbAfRRRoWBH4IWJARJvS/tO92mlab8S+9NAnsc2DCM3EIDgeHCgdlV+W2xdERpD0g/bDqY42zskjMnC+A9RebotqH3FZxBJtuW+icRb4I3E0s4CMOTWmpHvsHsIYuEsh4j87q/PjZxP2Ft99IVNNrRdVtGTTKh7nMShfRC0KcSn3pjy48FOj+aTuftmG1R/nCLd57vyq37VhRUY07JTYQvXgy9MNQRlrEpV+a1fLJPqDCuVj1364q0H6Pvk0jdopLvx3w9emhYuqJSPNVXtO7vsccL2TQj84lOBglWB2AnuH0BkXNBoaCBFFxLwM/VPq9B1JXwH";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
    }

    private void getLocation() {

        // Load the data sets that for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsRoverRuckus);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * This Rover Ruckus sample places a specific target in the middle of each perimeter wall.
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        /**
         * To place the BlueRover target in the middle of the blue perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Then, we translate it along the Y axis to the blue perimeter wall.
         */
        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        /**
         * To place the RedFootprint target in the middle of the red perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 180 around the field's Z axis so the image is flat against the red perimeter wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative Y axis to the red perimeter wall.
         */
        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        /**
         * To place the FrontCraters target in the middle of the front perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 90 around the field's Z axis so the image is flat against the front wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative X axis to the front perimeter wall.
         */
        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        /**
         * To place the BackSpace target in the middle of the back perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it -90 around the field's Z axis so the image is flat against the back wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the X axis to the back perimeter wall.
         */
        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        /**
         * Create a transformation matrix describing where the phone is on the robot.
         *
         * The coordinate frame for the robot looks the same as the field.
         * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
         * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
         *
         * The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
         * pointing to the LEFT side of the Robot.  It's very important when you test this code that the top of the
         * camera is pointing to the left side of the  robot.  The rotation angles don't work if you flip the phone.
         *
         * If using the rear (High Res) camera:
         * We need to rotate the camera around it's long axis to bring the rear camera forward.
         * This requires a negative 90 degree rotation on the Y axis
         *
         * If using the Front (Low Res) camera
         * We need to rotate the camera around it's long axis to bring the FRONT camera forward.
         * This requires a Positive 90 degree rotation on the Y axis
         *
         * Next, translate the camera lens to where it is on the robot.
         * In this example, it is centered (left to right), but 110 mm forward of the middle of the robot, and 200 mm above ground level.
         */

        final int CAMERA_FORWARD_DISPLACEMENT  = 110;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables)
        {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, this.parameters.cameraDirection);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        /** Start tracking the data sets we care about. */
        targetsRoverRuckus.activate();
        while (opModeIsActive()) {

            // check all the trackable target to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                break;
            }
            else {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();
        }
    }

    private void rotateRobot(int degrees){
        double direction = 1.0;
        double power = 0.25;
        int secPerDegree = 35;
        if (degrees > 0){
            direction = -1.0;
        }
        leftDrive.setPower(direction*power);
        rightDrive.setPower(direction*power*-1.0);
        sleep(Math.abs(secPerDegree*degrees));
        leftDrive.setPower(0.0);
        rightDrive.setPower(0.0);
    }

    private void InitGyro () {
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

        while (true){
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double heading = (angles.firstAngle+360+0)%360;
            sleep(100);
            telemetry.addData("Angle", Double.toString(heading));
            telemetry.update();
        }

    }
}

