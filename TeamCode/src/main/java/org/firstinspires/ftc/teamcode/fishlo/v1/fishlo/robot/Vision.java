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

    private OpenCvCamera camera;
    private boolean isUsingWebcam = true;
    private String cameraName = "webcam";
    private VisionPipeline pipeline;

    private int WIDTH = 432;
    private int HEIGHT = 240;
    private double thresholdRight = 2 * WIDTH / 3.0;
    private double thresholdLeft = WIDTH / 3.0;

    public enum DetectorState {
        NOT_CONFIGURED,
        INITIALIZING,
        RUNNING,
        INIT_FAILURE_NOT_RUNNING;
    }

    private DetectorState detectorState = DetectorState.NOT_CONFIGURED;

    private final Object sync = new Object();

    public enum position {
        LEFT,
        RIGHT,
        CENTER,
        NULL
    }

    public Vision(Robot robot) {
        super(robot);
    }

    @Override
    public void init() {
        synchronized (sync) {
            if (detectorState == DetectorState.NOT_CONFIGURED) {
                //This will instantiate an OpenCvCamera object for the camera we'll be using
                int cameraMonitorViewId = robot.hardwareMap
                        .appContext.getResources()
                        .getIdentifier("cameraMonitorViewId", "id", robot.hardwareMap.appContext.getPackageName());
                if (isUsingWebcam) {
                    camera = OpenCvCameraFactory.getInstance()
                            .createWebcam(robot.hardwareMap.get(WebcamName.class, cameraName), cameraMonitorViewId);
                } else {
                    camera = OpenCvCameraFactory.getInstance()
                            .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
                }

                //Set the pipeline the camera should use and start streaming
                camera.setPipeline(pipeline = new VisionPipeline());


                detectorState = DetectorState.INITIALIZING;

                camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {

                        camera.startStreaming(WIDTH, HEIGHT, OpenCvCameraRotation.UPRIGHT);

                        synchronized (sync) {
                            detectorState = DetectorState.RUNNING;
                        }
                    }

                    public void onError(int errorCode) {

                        synchronized (sync) {
                            detectorState = DetectorState.INIT_FAILURE_NOT_RUNNING; //Set our state
                        }

                        RobotLog.addGlobalWarningMessage("Warning: Camera device failed to open with EasyOpenCv error: " +
                                ((errorCode == -1) ? "CAMERA_OPEN_ERROR_FAILURE_TO_OPEN_CAMERA_DEVICE" : "CAMERA_OPEN_ERROR_POSTMORTEM_OPMODE")
                        ); //Warn the user about the issue
                    }
                });
            }
        }
    }

    @Override
    public void handle() {}

    @Override
    public void stop() {
        camera.stopStreaming();
    }

    public OpenCvCamera getCamera() {
        return camera;
    }

    public void setLowerBound(Scalar low) {
        pipeline.setLowerBound(low);
    }

    public void setUpperBound(Scalar high) {
        pipeline.setUpperBound(high);
    }

    public void setLowerAndUpperBounds(Scalar low, Scalar high) {
        pipeline.setLowerAndUpperBounds(low, high);
    }

    // The area below thresholdLeft will be the Left placement, to the right of
    // thresholdRight will be Right placement, and the area between those is the center
    // 0.0 to 1.0
    public void setPercentThreshold(double percentLeft, double percentRight) {
        thresholdLeft = WIDTH * percentLeft;
        thresholdRight = WIDTH * percentRight;
    }

    // 0.0 to WIDTH
    public void setThreshold(double pixelsLeft, double pixelsRight) {
        thresholdLeft = pixelsLeft;
        thresholdLeft = pixelsRight;
    }

    public enum Placement {
        LEFT,
        RIGHT,
        CENTER
    }
    
    public Placement getPlacement() {
        if (pipeline.getCentroid() != null) {
            if (pipeline.getCentroid().x > thresholdRight)
                return Placement.RIGHT;
            else if (pipeline.getCentroid().x < thresholdLeft)
                return Placement.LEFT;
        }
        return Placement.CENTER;
    }

}
