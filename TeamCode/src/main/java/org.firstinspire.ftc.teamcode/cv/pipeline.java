package org.firstinspire.ftc.teamcode.cv;


import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class pipeline extends OpenCvPipeline {
    Mat resizedToWidthHeight = new Mat();
    Mat resizedWithMultiplier = new Mat();
    Mat mask = new Mat();
    Mat ret = new Mat();
    @Override
    public Mat processFrame(Mat img) {
        int width = 100;
        int height = 100;
        Imgproc.resize(img, resizedToWidthHeight, new Size(width, height));
        double xMult = 2;
        double yMult = 0.5;
        Imgproc.resize(img, resizedWithMultiplier, new Size(0,0), xMult, yMult);
        Imgproc.cvtColor(img, img, Imgproc.COLOR_RGB2GRAY);
        Imgproc.threshold(img, mask, 200, 255, Imgproc.THRESH_BINARY);
        Core.bitwise_and(img, mask, ret);
        return ret;
    }
}
