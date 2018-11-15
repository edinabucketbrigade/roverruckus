/* Copyright (c) 2018 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.test;

import com.edinaftcrobotics.vision.camera.BackPhoneCamera;
import com.edinaftcrobotics.vision.camera.Camera;
import com.edinaftcrobotics.vision.tracker.roverruckus.GoldMineralTracker;
import com.edinaftcrobotics.vision.tracker.roverruckus.PictureTracker;
import com.edinaftcrobotics.vision.utils.Triple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;


/**
 * This 2018-2019 OpMode illustrates the basics of using the Vuforia localizer to determine
 * positioning and orientation of robot on the FTC field.
 * The code is structured as a LinearOpMode
 *
 * Vuforia uses the phone's camera to inspect it's surroundings, and attempt to locate target images.
 *
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code than combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 *
 * This example assumes a "square" field configuration where the red and blue alliance stations
 * are on opposite walls of each other.
 *
 * From the Audience perspective, the Red Alliance station is on the right and the
 * Blue Alliance Station is on the left.

 * The four vision targets are located in the center of each of the perimeter walls with
 * the images facing inwards towards the robots:
 *     - BlueRover is the Mars Rover image target on the wall closest to the blue alliance
 *     - RedFootprint is the Lunar Footprint target on the wall closest to the red alliance
 *     - FrontCraters is the Lunar Craters image target on the wall closest to the audience
 *     - BackSpace is the Deep Space image target on the wall farthest from the audience
 *
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

@TeleOp(name="Test: Picture Tracker Phone", group ="Teleop Test")
public class PictureTrackerPhoneTest extends LinearOpMode {

    static {
        System.loadLibrary("opencv_java3");
    }


//    @Override public void runOpMode2() throws InterruptedException {
//        ElapsedTime stopwatch = new ElapsedTime();
//        Camera camera = new BackPhoneCamera();
//
//        camera.activate();
//
//        PictureTracker pictureTracker = null;
//        pictureTracker = new PictureTracker(camera, 110, 200, 0);
//
//        pictureTracker.startTracking();
//        waitForStart();
//
//        stopwatch.reset();
//
//        /** Start tracking the data sets we care about. */
//        while (opModeIsActive()) {
//            Triple trackableObject = pictureTracker.getTrackableObject(telemetry);
//
//            if (trackableObject != null) {
//                telemetry.addData("Visible Target", trackableObject.PictureName);
//                telemetry.addData("Pos (in) ", "{X, Y, Z} = %.1f, %.1f, %.1f",
//                        trackableObject.Point.x, trackableObject.Point.y, trackableObject.Point.z);
//                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f}", trackableObject.Orientation.firstAngle,
//                        trackableObject.Orientation.secondAngle, trackableObject.Orientation.thirdAngle);
//            } else {
//                telemetry.addData("Picture", "not found");
//            }
//            telemetry.update();
//            idle();
//        }
//
//        pictureTracker.stopTracking();
//        camera.deactivate();
//    }

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime stopwatch = new ElapsedTime();
        Camera camera = new BackPhoneCamera();

        camera.activate();
        PictureTracker pictureTracker = new PictureTracker(camera,0,0,0);

        pictureTracker.startTracking();

        waitForStart();
        stopwatch.reset();

        String desiredPictureName = "Blue-Rover";

        /** Start tracking the data sets we care about. */
        while (opModeIsActive()) {



            Triple location = pictureTracker.getTrackableObject(telemetry);



            if (location != null) {
                // it could not see anything
                // spin a little more

                telemetry.addData("OBJECT FOUND: ", "%f %f %f", location.Orientation.firstAngle, location.Orientation.secondAngle, location.Orientation.thirdAngle);

                // is it the picture we want?
                if (desiredPictureName == location.PictureName) {

                    // hooray drive straight towards it
                    telemetry.addData("DESIRED OBJECT FOUND: ", "%f %f %f", location.Orientation.firstAngle, location.Orientation.secondAngle, location.Orientation.thirdAngle);
                    telemetry.addData("POINTS", "X%f Y%f Z%f ", location.Point.x, location.Point.y, location.Point.z);

                }

            }

            // spin some more
            telemetry.addData("Spinning Nothing found", null);

            sleep(250);



            telemetry.update();

        }

        camera.deactivate();
    }
}
