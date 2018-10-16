package com.edinaftcrobotics.vision.camera;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class WebCamCamera extends Camera {
    public WebCamCamera(HardwareMap hardwareMap) {
        super();

        _cameratype = CameraType.Web;

        WebcamName webcamName = hardwareMap.get (WebcamName.class, "Webcam 1");

        _params.cameraName = webcamName;
    }
}
