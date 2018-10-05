package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Drive_Forward")
public class DriveForward extends LinearOpMode {


    HardwareBucketBrigade hw = new HardwareBucketBrigade();
    ElapsedTime timer = new ElapsedTime();


    final static int RED = -1;
    final static int BLUE = 1;
    final static int FORWARD = 1;
    final static int BACKWARD = -1;
    int teamColor = RED;

    double jewelSpeed = 0.5;
    final int TIME_TO_KNOCK_OFF = 3000; // ms


    public void driveDirection(int forwardOrBackward) {
  /*      hw.leftDrive.setPower(jewelSpeed * forwardOrBackward);
        hw.rightDrive.setPower(jewelSpeed * forwardOrBackward); */
        telemetry.addData("Status", "In drive direction");
    }

    int colorSeen;
    @Override
    public void runOpMode() {

        int driveDirection = teamColor;
        driveDirection(driveDirection);




    }


}