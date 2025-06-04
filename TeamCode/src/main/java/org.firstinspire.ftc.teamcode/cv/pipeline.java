package org.firstinspire.ftc.teamcode.cv;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class pipeline extends OpenCvPipeline {
    @Override
    public Mat processFrame(Mat input) {
        return input;
    }
}
