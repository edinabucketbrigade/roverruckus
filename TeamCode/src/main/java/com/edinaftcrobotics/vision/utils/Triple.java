package com.edinaftcrobotics.vision.utils;

import android.graphics.drawable.GradientDrawable;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.opencv.core.Point;
import org.opencv.core.Point3;

public class Triple {
    public String PictureName;
    public Point3 Point;
    public Orientation Orientation;

    public Triple(String pictureName, Point3 point, Orientation orientation) {
        PictureName = pictureName;
        Point = point;
        Orientation = orientation;
    }
}
