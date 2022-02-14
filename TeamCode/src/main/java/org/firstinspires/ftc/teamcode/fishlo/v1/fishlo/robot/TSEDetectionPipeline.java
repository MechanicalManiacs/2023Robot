package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Range;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class TSEDetectionPipeline extends OpenCvPipeline {

    static final Rect LEFT_ROI = new Rect(
            new Point( 0, 0 ),
            new Point( 426, 720 ) );
    static final Rect MIDDLE_ROI = new Rect(
            new Point( 426, 0 ),
            new Point( 852, 720 ) );
    static final Rect RIGHT_ROI = new Rect(
            new Point( 852, 0 ),
            new Point( 1278, 720 ) );

    public Scalar HSVUpper = new Scalar(0, 0, 0);
    public Scalar HSVLower = new Scalar(0, 0, 0);

    public String getPos() {
        return pos;
    }

    private String pos;

    public Scalar getHSVUpper() {
        return HSVUpper;
    }

    public void setHSVUpper(Scalar HSVUpper) {
        this.HSVUpper = HSVUpper;
    }

    public Scalar getHSVLower() {
        return HSVLower;
    }

    public void setHSVLower(Scalar HSVLower) {
        this.HSVLower = HSVLower;
    }

    public void setPos(String pos) {
        this.pos = pos;
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat blurredImage = new Mat();
        Mat hsvImage = new Mat();
        Mat mask = new Mat();
        Mat morphOutput = new Mat();

        //remove noise
        Imgproc.GaussianBlur(input, blurredImage, new Size(7, 7), 0);

        //convert to HSV
        Imgproc.cvtColor(blurredImage, hsvImage, Imgproc.COLOR_BGR2HSV);

        //threshold image to HSV values
        Core.inRange(hsvImage, HSVLower, HSVUpper, mask);

        //morhpological operators
        Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(24, 24));
        Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(12, 12));

        Imgproc.erode(mask, morphOutput, erodeElement);
        Imgproc.erode(mask, morphOutput, erodeElement);

        Imgproc.dilate(mask, morphOutput, dilateElement);
        Imgproc.dilate(mask, morphOutput, dilateElement);

        input = getContours(mask, input);

        return input;

    }

    public Mat getContours(Mat maskedImage, Mat input) {
        Mat left = maskedImage.submat(LEFT_ROI);
        Mat center = maskedImage.submat(MIDDLE_ROI);
        Mat right = maskedImage.submat(RIGHT_ROI);

        //find contours
        List<MatOfPoint> contoursleft = new ArrayList<>();
        List<MatOfPoint> contoursright = new ArrayList<>();
        List<MatOfPoint> contourscenter = new ArrayList<>();
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();

        Imgproc.findContours(left, contoursleft, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(center, contourscenter, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(right, contoursright, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);

        if (contoursleft.isEmpty() && contourscenter.isEmpty() && !contoursright.isEmpty()) {
            contours = contoursright;
            pos = "Right";
        }
        else if (!contoursleft.isEmpty() && contourscenter.isEmpty() && contoursright.isEmpty()) {
            contours = contoursleft;
            pos = "Left";
        }
        else if (contoursleft.isEmpty() && !contourscenter.isEmpty() && contoursright.isEmpty()) {
            contours = contourscenter;
            pos = "Center";
        }

        if (hierarchy.size().height > 0 && hierarchy.size().width > 0) {
            for (int i = 0; i >= 0; i = (int) hierarchy.get(0, i)[0]) {
                Imgproc.drawContours(input, contours, i, new Scalar(0, 255, 0));
            }
        }

        return input;
    }

}
