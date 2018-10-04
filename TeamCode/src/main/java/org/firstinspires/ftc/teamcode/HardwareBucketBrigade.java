package org.firstinspires.ftc.teamcode;

/**
 * Created by phinn on 11/18/2017.
 */
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/*
    Physical decsription and configuration of robot
 */
public class HardwareBucketBrigade {

    public DcMotor leftDrive;
    public DcMotor rightDrive;
    public DcMotor strafeWheel;
    public DcMotor upDown;
    //   public ColorSensor colorSensor;

    public void init(HardwareMap hardwareMap) {

        leftDrive = hardwareMap.get(DcMotor.class, "leftWheel");
        rightDrive = hardwareMap.get(DcMotor.class, "rightFront");
        strafeWheel = hardwareMap.get(DcMotor.class, "strafeWheel");
        upDown = hardwareMap.get(DcMotor.class, "upDown");
//        colorSensor = hardwareMap.get(ColorSensor.class, "color");


        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive.setDirection(DcMotor.Direction.FORWARD);

        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        strafeWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        strafeWheel.setDirection(DcMotor.Direction.FORWARD);

        upDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        upDown.setDirection(DcMotor.Direction.FORWARD);

        ///colorServo.scaleRange(this.COLOR_SERVO_MIN_POSITION, this.COLOR_SERVO_MAX_POSITION);
    }

    public static final double LEFT_CALIBRATION = 1.0;
    public static final double RIGHT_CALIBRATION = 1.0;


    public static final double MAX_RATIO = 0.75;

    void setMotorSpeed(double leftStick, double rightStick) {
        leftDrive.setPower(leftStick*MAX_RATIO*LEFT_CALIBRATION);
        rightDrive.setPower(rightStick*MAX_RATIO*RIGHT_CALIBRATION);

    }
}
