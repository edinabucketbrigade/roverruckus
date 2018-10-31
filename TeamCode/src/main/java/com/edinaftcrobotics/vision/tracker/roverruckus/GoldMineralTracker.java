package com.edinaftcrobotics.vision.tracker.roverruckus;

import android.graphics.Bitmap;

import com.edinaftcrobotics.vision.camera.Camera;
import com.edinaftcrobotics.vision.detector.roverruckus.GenericDetector;
import com.edinaftcrobotics.vision.detector.roverruckus.GoldAlignDetector;
import com.edinaftcrobotics.vision.filter.LeviColorFilter;
import com.edinaftcrobotics.vision.tracker.BaseTracker;
import com.edinaftcrobotics.vision.utils.Enums.AreaScoringMethod;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;

public class GoldMineralTracker extends BaseTracker {
    private GoldAlignDetector _goldDetector;


    public GoldMineralTracker(Camera camera) {
        _camera = camera;
        _camera.getPOCVuforia().setFrameQueueCapacity(1);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);

        _goldDetector = new GoldAlignDetector();

        _goldDetector.yellowFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.YELLOW, 100);
        _goldDetector.useDefaults();
        _goldDetector.areaScoringMethod = AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
    }

    public boolean getGoldMineralLocation() throws InterruptedException
    {
        VuforiaLocalizer.CloseableFrame frame = _camera.getPOCVuforia().getFrameQueue().take();
        Bitmap b = getVuforiaImage(frame, PIXEL_FORMAT.RGB565);

        if(b != null){
            Mat map = bitmapToMat(b, CvType.CV_8UC3);

            _goldDetector.processFrame(map);
            map.release();
        }

        frame.close();

        return _goldDetector.isFound();
    }

    public double getXPosition() { return _goldDetector.getXPosition(); }

    public double getYPosition() { return _goldDetector.getYPosition(); }

    public boolean aligned() { return _goldDetector.getAligned(); }

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
