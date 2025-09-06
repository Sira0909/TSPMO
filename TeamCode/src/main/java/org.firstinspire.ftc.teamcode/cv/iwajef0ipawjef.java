package org.firstinspire.ftc.teamcode.cv;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

    public class iwajef0ipawjef extends OpenCvPipeline {
        Mat mask = new Mat();
        Mat ret = new Mat();
        Scalar red = new Scalar(255, 0, 0);
        @Override
        public Mat processFrame(Mat img) {
            int width = img.cols();
            int height = img.rows();

            // Draw the size as text in the top-left corner
            Imgproc.putText(
                    img,
                    "Size: " + width + " x " + height,
                    new Point(10, 30),
                    Imgproc.FONT_HERSHEY_SIMPLEX,
                    1.0,                 // font scale
                    new Scalar(0, 0, 255), // color (BGR)
                    2                    // thickness
            );

            return img;
        }
    }
