package com.edinaftcrobotics.vision.camera;

import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;

public class POCVuforia extends VuforiaLocalizerImpl {
    public POCVuforia(Parameters parameters) { super(parameters); }

    public void closeAll() { super.close(); }
}
