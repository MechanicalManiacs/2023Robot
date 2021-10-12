package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.objdetect.QRCodeDetector;

public class VisionPipeline extends OpenCvPipeline {

    private Mat matYCrCb = new Mat();

    private Mat points = new Mat();

    private QRCodeDetector detector = new QRCodeDetector();

    private String data = new String();

    private Point topLeft;

    @Override
    public Mat processFrame(Mat input) {
        data = detector.detectAndDecode(input);
        return input;
    }

    public String getData() {
        return data;
    }

    public Point getPoint() {
        return topLeft;
    }

}
