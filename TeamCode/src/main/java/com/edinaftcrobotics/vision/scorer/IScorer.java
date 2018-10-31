package com.edinaftcrobotics.vision.scorer;

import org.opencv.core.Mat;

/**
 * Created by Victo on 9/10/2018.
 */
public interface IScorer {
    double calculateScore(Mat input);
}
