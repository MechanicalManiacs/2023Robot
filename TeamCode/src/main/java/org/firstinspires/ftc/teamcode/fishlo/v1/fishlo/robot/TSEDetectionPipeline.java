package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class TSEDetectionPipeline extends OpenCvPipeline {

    double hue = 0;
    double sens = 0;

    double leftThreshold = 233;
    double rightThreshold = 383;

    List<MatOfPoint> contours = new ArrayList<>();

    Mat blur = new Mat();
    Mat hsv = new Mat();
    Mat single = new Mat();
    Mat hierarchy = new Mat();
    Mat output = new Mat();

    Rect largestRect;

    public BarcodePosition barcodePosition = BarcodePosition.NULL;

    public enum BarcodePosition {
        LEFT,
        RIGHT,
        CENTER,
        NULL
    }

    @Override
    public Mat processFrame(Mat input) {
        input.copyTo(output);

        //cutting off top of the image so that it only detect capstone
        output = output.submat(output.rows() / 3, output.rows()-1, 0, output.cols()-1);

        //Blur
        Imgproc.GaussianBlur(output, blur, new Size(5, 5), 0);

        //Convert to HSV
        Imgproc.cvtColor(blur, hsv, Imgproc.COLOR_RGB2HSV);

        contours.clear();

        hue = 50;
        sens = 30;

        //Find contours, (orange)
        Scalar lowHSV = new Scalar((hue/2)-sens, 60 , 85);
        Scalar highHSV = new Scalar(hue+sens, 255, 255);
        Core.inRange(hsv, lowHSV, highHSV, single);
        Imgproc.findContours(single, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        //checks if contours are found
        if (contours.size() > 0) {
            //finds biggest contour
            double max = 0;
            int ind = 0;
            for (int i = 0; i < contours.size(); i++) {
                double area = Imgproc.contourArea(contours.get(i));
                if (area > max) {
                    ind = i;
                    max = area;
                }
            }
            //creates rectangle from the contour and gets the x value
            largestRect = Imgproc.boundingRect(contours.get(ind));
            Scalar boxColor = new Scalar(0, 255, 0);
            Imgproc.rectangle(output, largestRect, boxColor, 3, 8, 0);
            Imgproc.putText(output, "X: " + getRectX() + " Y: " + getRectY(), new Point(5, output.height() - 5), 0, 0.6, new Scalar(255, 255, 255));
            Imgproc.drawContours(output, contours, -1, new Scalar(255, 0, 0));

            //determine barcode position
            if (largestRect.x > rightThreshold) {
                barcodePosition = BarcodePosition.RIGHT;
            }
            else if (largestRect.x < leftThreshold) {
                barcodePosition = BarcodePosition.LEFT;
            }
            else if (largestRect.x > leftThreshold && largestRect.x < rightThreshold) {
                barcodePosition = BarcodePosition.CENTER;
            }
            else {
                barcodePosition = BarcodePosition.NULL;
            }
        }

        return output;
    }

    public BarcodePosition getBarcodePosition() {
        return  barcodePosition;
    }

    public int getRectX() {
        return largestRect.x;
    }

    public int getRectY() {
        return largestRect.y;
    }
}