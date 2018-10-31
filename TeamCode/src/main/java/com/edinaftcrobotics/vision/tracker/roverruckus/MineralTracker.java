package com.edinaftcrobotics.vision.tracker.roverruckus;

import android.graphics.Bitmap;

import org.opencv.core.Point;
import org.opencv.core.Rect;

import com.edinaftcrobotics.vision.camera.Camera;
import com.edinaftcrobotics.vision.tracker.BaseTracker;
import com.edinaftcrobotics.vision.detector.roverruckus.GenericDetector;
import com.edinaftcrobotics.vision.filter.LeviColorFilter;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import com.vuforia.Image;
import org.opencv.android.Utils;

import com.edinaftcrobotics.vision.utils.Enums.*;

public class MineralTracker extends BaseTracker {
    private GenericDetector _genericDetector;


    public MineralTracker(Camera camera) {
        _camera = camera;
        _camera.getPOCVuforia().setFrameQueueCapacity(1);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);

        _genericDetector = new GenericDetector();
        _genericDetector.colorFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.YELLOW);

        //Jewel Detector Settings
        _genericDetector.areaWeight = 0.02;
        _genericDetector.detectionMode = AreaScoringMethod.MAX_AREA; // PERFECT_AREA
        //genericDeterctor.perfectArea = 6500; <- Needed for PERFECT_AREA
        _genericDetector.debugContours = true;
        _genericDetector.maxDiffrence = 15;
        _genericDetector.ratioWeight = 15;
        _genericDetector.minArea = 100;
    }

    public Rect getGoldMineralLocation() throws InterruptedException
    {
        Rect mineralLocation = null;
        VuforiaLocalizer.CloseableFrame frame = _camera.getPOCVuforia().getFrameQueue().take();
        Bitmap b = getVuforiaImage(frame, PIXEL_FORMAT.RGB565);

        if(b != null){
            Mat map = bitmapToMat(b, CvType.CV_8UC3);

            _genericDetector.processFrame(map);
            mineralLocation = _genericDetector.getRect();
            map.release();
        }

        frame.close();

        return mineralLocation;
    }

    public Rect getLastMineralRectangle() { return _genericDetector.getRect(); }

    public Point getLastMineralLocation() { return _genericDetector.getLocation(); }

    private Bitmap getVuforiaImage(VuforiaLocalizer.CloseableFrame frame, int format){
        Image img;
        long numImgs = frame.getNumImages();
        for (int i = 0; i < numImgs; i++) {
            if (frame.getImage(i).getFormat() == format) {
                img =  frame.getImage(i);
                Bitmap bm = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
                bm.copyPixelsFromBuffer(img.getPixels());
                return bm;
            }//if
        }//for
        return null;
    }

    private Mat bitmapToMat (Bitmap bit, int cvType) {
        Mat newMat = new Mat(bit.getHeight(), bit.getWidth(), cvType);

        Utils.bitmapToMat(bit, newMat);

        return newMat;
    }
}
