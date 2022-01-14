package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class Vision extends SubSystem {

    private OpenCvCamera webcam;
    private VisionPipeline pipeline;

    private double crThreshHigh = 150;
    private double crThreshLow = 120;
    private double cbThreshHigh = 255;
    private double cbThreshLow = 255;

    private int minRectangleArea = 2000;
    private double leftBarcodeRangeBoundary = 0.32; //i.e 30% of the way across the frame from the left
    private double rightBarcodeRangeBoundary = 0.82; //i.e 60% of the way across the frame from the left

    // Pink Range                                      Y      Cr     Cb
    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 150.0, 20.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 100.0);

    /**
     * Construct a subsystem with the robot it applies to.
     *
     * @param robot
     */
    public Vision(Robot robot) {
        super(robot);
    }

    @Override
    public void init() {
        // OpenCV webcam
        int cameraMonitorViewId = robot.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robot.hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(robot.hardwareMap.get(WebcamName.class, "webcam 1"), cameraMonitorViewId);
        //OpenCV Pipeline

        pipeline = new VisionPipeline(0.005, 0.005, 0.05, 0.005);

        pipeline.configureScalarLower(scalarLowerYCrCb.val[0],scalarLowerYCrCb.val[1],scalarLowerYCrCb.val[2]);
        pipeline.configureScalarUpper(scalarUpperYCrCb.val[0],scalarUpperYCrCb.val[1],scalarUpperYCrCb.val[2]);

        webcam.setPipeline(pipeline);

        // Webcam Streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    public String getPlacement() {
        String placement = "";
        if(pipeline.error){
            robot.telemetry.addData("Exception: ", pipeline.debug.getStackTrace());
        }
        // Only use this line of the code when you want to find the lower and upper values, using Ftc Dashboard (https://acmerobotics.github.io/ftc-dashboard/gettingstarted)
        // testing(pipeline);

        // Watch our YouTube Tutorial for the better explanation

        double rectangleArea = pipeline.getRectArea();

        //Print out the area of the rectangle that is found.
        robot.telemetry.addData("Midpoint", pipeline.getRectMidpointX());

        //Check to see if the rectangle has a large enough area to be a marker.
        if(rectangleArea > minRectangleArea){
            //Then check the location of the rectangle to see which barcode it is in.
            if(pipeline.getRectMidpointX() > /*rightBarcodeRangeBoundary * pipeline.getRectWidth()*/ 400){
                robot.telemetry.addData("Barcode Position", "Right");
                placement = "Right";
            }
            else if(pipeline.getRectMidpointX() < /*leftBarcodeRangeBoundary * pipeline.getRectWidth()*/ 150){
                robot.telemetry.addData("Barcode Position", "Left");
                placement = "Left";
            }
            else {
                robot.telemetry.addData("Barcode Position", "Center");
                placement = "Center";
            }
        }

        robot.telemetry.update();
        return placement;
    }

    @Override
    public void handle() {

    }

    @Override
    public void stop() {
        webcam.stopStreaming();
    }
}
