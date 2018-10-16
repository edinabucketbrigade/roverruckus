package com.edinaftcrobotics.vision.camera;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

public class BackPhoneCamera extends Camera {
    public BackPhoneCamera() {
        super();

        _cameratype = CameraType.Back;

        _params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
    }
}
