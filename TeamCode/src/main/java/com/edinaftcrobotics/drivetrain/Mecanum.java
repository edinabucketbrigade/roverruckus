package com.edinaftcrobotics.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Mecanum {
    private DcMotor _frontLeft;
    private DcMotor _frontRight;
    private DcMotor _backLeft;
    private DcMotor _backRight;

    public Mecanum(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br)
    {
        _frontLeft = fl;
        _frontRight = fr;
        _backLeft = bl;
        _backRight = br;

        _backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        _frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void SlideLeft(double power){
        _frontLeft.setPower(power);
        _frontRight.setPower(-power);
        _backLeft.setPower(-power);
        _backRight.setPower(power);
    }

    public void SlideRight(double power){
        _frontLeft.setPower(-power);
        _frontRight.setPower(power);
        _backLeft.setPower(power);
        _backRight.setPower(-power);
    }

    public void MoveSW(double power) { // SW
        _frontLeft.setPower(power);
        _frontRight.setPower(0);
        _backLeft.setPower(0);
        _backRight.setPower(power);
    }

    public void MoveNW(double power) { // NW
        _frontLeft.setPower(0);
        _frontRight.setPower(-power);
        _backLeft.setPower(-power);
        _backRight.setPower(0);
    }

    public void MoveSE(double power) { // SE
        _frontLeft.setPower(0);
        _frontRight.setPower(power);
        _backLeft.setPower(power);
        _backRight.setPower(0);
    }

    public void MoveNE(double power) { // NE
        _frontLeft.setPower(-power);
        _frontRight.setPower(0);
        _backLeft.setPower(0);
        _backRight.setPower(-power);
    }

    public void MoveForward(double power) {
        _frontLeft.setPower(-power);
        _frontRight.setPower(-power);
        _backLeft.setPower(-power);
        _backRight.setPower(-power);
    }

    public void MoveBackwards(double power) {
        _frontLeft.setPower(power);
        _frontRight.setPower(power);
        _backLeft.setPower(power);
        _backRight.setPower(power);
    }

    public void TurnRight(double power) {
        _frontLeft.setPower(-power);
        _frontRight.setPower(power);
        _backLeft.setPower(-power);
        _backRight.setPower(power);
    }

    public void TurnLeft(double power) {
        _frontLeft.setPower(power);
        _frontRight.setPower(-power);
        _backLeft.setPower(power);
        _backRight.setPower(-power);
    }

    public void Stop() {
        _frontLeft.setPower(0);
        _frontRight.setPower(0);
        _backLeft.setPower(0);
        _backRight.setPower(0);
    }

    public void Drive(double leftStickX, double leftStickY, double rightStickY) {
        final double x = Math.pow(-leftStickX, 3.0);
        final double y = Math.pow(leftStickY, 3.0);

        final double rotation = Math.pow(-rightStickY, 3.0);
        final double direction = Math.atan2(x, y);
        final double speed = Math.min(1.0, Math.sqrt(x * x + y * y)) * 1.4;

        final double fl = speed * Math.sin(direction + Math.PI / 4.0) + rotation;
        final double fr = speed * Math.cos(direction + Math.PI / 4.0) - rotation;
        final double bl = speed * Math.cos(direction + Math.PI / 4.0) + rotation;
        final double br = speed * Math.sin(direction + Math.PI / 4.0) - rotation;

        _frontLeft.setPower(fl);
        _frontRight.setPower(fr);
        _backLeft.setPower(bl);
        _backRight.setPower(br);
    }
}
