package com.edinaftcrobotics.vision.camera;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

public class FrontPhoneCamera extends Camera {
    public  FrontPhoneCamera() {
        super ();

        _cameratype = CameraType.Front;

        _params.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
    }
}
