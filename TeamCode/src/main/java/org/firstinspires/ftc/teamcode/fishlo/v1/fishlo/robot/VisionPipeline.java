package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot;

import org.apache.commons.math3.fraction.Fraction;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.objdetect.QRCodeDetector;

public class VisionPipeline extends OpenCvPipeline {

    private Mat points = new Mat();

    private QRCodeDetector detector = new QRCodeDetector();

    private String data = new String();

    private Point center;

    protected int imageWidth;
    protected int imageHeight;

    private double fov;
    private double horizontalFocalLength;
    private double verticalFocalLength;

    public VisionPipeline(double fov) {
        this.fov = fov;
    }

    @Override
    public void init(Mat mat) {
        super.init(mat);

        imageWidth = mat.width();
        imageHeight = mat.height();

        double diagonalView = Math.toRadians(this.fov);
        Fraction aspectFraction = new Fraction(this.imageWidth, this.imageHeight);
        int horizontalRatio = aspectFraction.getNumerator();
        int verticalRatio = aspectFraction.getDenominator();
        double diagonalAspect = Math.hypot(horizontalRatio, verticalRatio);
        double horizontalView = Math.atan(Math.tan(diagonalView / 2) * (horizontalRatio / diagonalAspect)) * 2;
        horizontalFocalLength = this.imageWidth / (2 * Math.tan(horizontalView / 2));
    }

    @Override
    public Mat processFrame(Mat input) {
        data = detector.detectAndDecode(input, points);
        if (points.empty()) {
            data = "Not Found";
        }
        else if (!points.empty()){
            center = new Point(points.get(points.rows() / 2, points.cols() / 2));
            for (int i = 0; i < points.cols(); i++) {
                Point p1 = new Point(points.get(0, i));
                Point p2 = new Point(points.get(0, (i + 1) % 4));
                Imgproc.line(input, p1, p2, new Scalar(0, 69, 255), 4);
            }
        }
        return input;
    }

    public String getData() {
        return data;
    }

    public Point getCenter() {
        return center;
    }

    public double getAngle(Point point, double offsetCenterX) {
        double targetCenterX = point.x;
        return Math.toDegrees(Math.atan((targetCenterX - offsetCenterX) / horizontalFocalLength));
    }

}
