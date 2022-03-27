package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

// Credits to team 7303 RoboAvatars, adjusted by team 3954 Pink to the Future

public class VisionPipeline extends OpenCvPipeline {
    Scalar ORANGE = new Scalar(255, 145, 0);

    // Pink, the default color                         Y      Cr     Cb    (Do not change Y)
     private static int hue = 50;
     private static int sens = 30;
     public static Scalar scalarLowerHSV = new Scalar((hue / 2) - sens, 60, 85);
     public static Scalar scalarUpperHSV = new Scalar(hue + sens, 255, 255);

    // Green                                             Y      Cr     Cb
    // public static Scalar scalarLowerYCrCb = new Scalar(  0.0, 0.0, 0.0);
    // public static Scalar scalarUpperYCrCb = new Scalar(255.0, 120.0, 120.0);
    // use this picture for you own color https://raw.githubusercontent.com/PinkToTheFuture/OpenCV_FreightFrenzy_2021-2022/main/7e8azlgi.bmp
    // Note that the Cr and Cb values range between 0-255. this means that the origin of the coordinate system is (128,128)

    //Volatile bc accessed by opmode without sync
    public volatile boolean error = false;
    public volatile Exception debug;

    private double borderLeftX = 0.0;   //fraction of pixels from the left side of the cam to skip
    private double borderRightX = 0.0;   //fraction of pixels from the right of the cam to skip
    private double borderTopY = 0.0;   //fraction of pixels from the top of the cam to skip
    private double borderBottomY = 0.0;   //fraction of pixels from the bottom of the cam to skip

    private int CAMERA_WIDTH;
    private int CAMERA_HEIGHT;

    private int loopCounter = 0;
    private int pLoopCounter = 0;

    private Mat mat = new Mat();
    private Mat processed = new Mat();
    private Mat output = new Mat();

    private Rect maxRect = new Rect(600,1,1,1);
    private Rect rect = new Rect(600,1,1,1);

    private double maxArea = 0;
    private boolean first = false;

    private final Object sync = new Object();

    public VisionPipeline(double borderLeftX, double borderRightX, double borderTopY, double borderBottomY) {
        this.borderLeftX = borderLeftX;
        this.borderRightX = borderRightX;
        this.borderTopY = borderTopY;
        this.borderBottomY = borderBottomY;
    }

    public void configureScalarLower(double y, double cr, double cb) {
        scalarLowerHSV = new Scalar(y, cr, cb);
    }

    public void configureScalarUpper(double y, double cr, double cb) {
        scalarUpperHSV = new Scalar(y, cr, cb);
    }

    public void configureScalarLower(int y, int cr, int cb) {
        scalarLowerHSV = new Scalar(y, cr, cb);
    }

    public void configureScalarUpper(int y, int cr, int cb) {
        scalarUpperHSV = new Scalar(y, cr, cb);
    }

    @Override
    public Mat processFrame(Mat input) {
        input = input.submat(input.rows() / 3, input.rows() - 1, 0, input.cols() - 1);
        CAMERA_WIDTH = input.width();
        CAMERA_HEIGHT = input.height();
        try {
            // Process Image
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
            Core.inRange(mat, scalarLowerHSV, scalarUpperHSV, processed);
            // Core.bitwise_and(input, input, output, processed);

            // GaussianBlur
            Imgproc.GaussianBlur(processed, processed, new Size(5.0, 5.0), 0.00);
            // Find Contours
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(processed, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

            // Draw Contours
            Imgproc.drawContours(input, contours, -1, new Scalar(255, 0, 0));

            //lock this up to prevent errors when outside threads access the max rect property.
            synchronized (sync) {
                // Loop Through Contours
                for (MatOfPoint contour : contours) {
                    Point[] contourArray = contour.toArray();

                    // Bound Rectangle if Contour is Large Enough
                    if (contourArray.length >= 15) {
                        MatOfPoint2f areaPoints = new MatOfPoint2f(contourArray);
                        Rect rect = Imgproc.boundingRect(areaPoints);

                        // if rectangle is larger than previous cycle or if rectangle is not larger than previous 6 cycles > then replace

                        if (rect.area() > maxArea
                                && rect.x > (borderLeftX * CAMERA_WIDTH) && rect.x + rect.width < CAMERA_WIDTH - (borderRightX * CAMERA_WIDTH)
                                && rect.y > (borderTopY * CAMERA_HEIGHT) && rect.y + rect.height < CAMERA_HEIGHT - (borderBottomY * CAMERA_HEIGHT)
                                || loopCounter - pLoopCounter > 6) {
                            maxArea = rect.area();
                            maxRect = rect;
                            pLoopCounter++;
                            loopCounter = pLoopCounter;
                            first = true;
                        }
                        areaPoints.release();
                    }
                    contour.release();
                }
                if (contours.isEmpty()) {
                    maxRect = new Rect();
                }
            }
            // Draw Rectangles If Area Is At Least 500
            if (first && maxRect.area() > 500) {
                Imgproc.rectangle(input, maxRect, new Scalar(0, 255, 0), 2);
            }
            // Draw Borders
            Imgproc.rectangle(input, new Rect(
                    (int) (borderLeftX * CAMERA_WIDTH),
                    (int) (borderTopY * CAMERA_HEIGHT),
                    (int) (CAMERA_WIDTH - (borderRightX * CAMERA_WIDTH) - (borderLeftX * CAMERA_HEIGHT)),
                    (int) (CAMERA_HEIGHT - (borderBottomY * CAMERA_WIDTH) - (borderTopY * CAMERA_HEIGHT))
            ), ORANGE, 2);

            // Display Data
            Imgproc.putText(input, "Area: " + getRectArea() + " Midpoint: " + getRectMidpointXY().x + " , " + getRectMidpointXY().y, new Point(5, CAMERA_HEIGHT - 5), 0, 0.6, new Scalar(255, 255, 255), 2);

            loopCounter++;
        } catch (Exception e) {
            debug = e;
            error = true;
        }
        return input;
    }
    /*
    Synchronize these operations as the user code could be incorrect otherwise, i.e a property is read
    while the same rectangle is being processed in the pipeline, leading to some values being not
    synced.
     */


    public int getRectHeight() {
        synchronized (sync) {
            return maxRect.height;
        }
    }

    public int getRectWidth() {
        synchronized (sync) {
            return maxRect.width;
        }
    }

    public int getRectX() {
        synchronized (sync) {
            return maxRect.x;
        }
    }

    public int getRectY() {
        synchronized (sync) {
            return maxRect.y;
        }
    }

    public double getRectMidpointX() {
        synchronized (sync) {
            return getRectX() + (getRectWidth() / 2.0);
        }
    }

    public double getRectMidpointY() {
        synchronized (sync) {
            return getRectY() + (getRectHeight() / 2.0);
        }
    }

    public Point getRectMidpointXY() {
        synchronized (sync) {
            return new Point(getRectMidpointX(), getRectMidpointY());
        }
    }

    public double getAspectRatio() {
        synchronized (sync) {
            return getRectArea() / (CAMERA_HEIGHT * CAMERA_WIDTH);
        }
    }

    public double getRectArea() {
        synchronized (sync) {
            return maxRect.area();
        }
    }

    public double getMaxRectWidth() {
        synchronized (sync) {
            return borderLeftX * CAMERA_WIDTH;
        }
    }

    public double getMaxRectHeight() {
        synchronized (sync) {
            return borderTopY * CAMERA_HEIGHT;
        }
    }
}