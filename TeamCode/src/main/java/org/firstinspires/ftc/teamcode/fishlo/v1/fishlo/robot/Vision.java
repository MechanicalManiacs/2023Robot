package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class Vision extends SubSystem {

    private OpenCvCamera camera;
    private String webcamName = "webcam";
    private VisionPipeline pipeline;
    private OpenCvCameraRotation ORIENTATION = OpenCvCameraRotation.UPRIGHT;

    private int fov = 78;

    public static int CAMERA_WIDTH = 320, CAMERA_HEIGHT = 240;

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
        int cameraMonitorViewId =
                robot.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robot.hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(robot.hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);

        camera.setPipeline(pipeline = new VisionPipeline(fov));
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, ORIENTATION);
            }
        });
    }

    @Override
    public void handle() {

    }

    @Override
    public void stop() {
        camera.stopStreaming();
    }

    public String getQRCodeData() {
        return pipeline.getData();
    }

    public boolean validateData(int match_number) {
        String data = getQRCodeData();
        return data.equals("16447-" + match_number);
    }

    public boolean validateData(boolean practice) {
        String data = getQRCodeData();
        return data.equals("16447");
    }

    public position getPosition() {
        position pos = position.CENTER;
        double angle = pipeline.getAngle(pipeline.getCenter(), 0);
        
        robot.telemetry.addData("point x", pipeline.getCenter().x);
        robot.telemetry.addData("point y", pipeline.getCenter().y);

        if (angle >= -39 && angle <= -13) {
            pos = position.LEFT;
        }
        else if (angle >= -13 && angle <= 13) {
            pos = position.CENTER;
        }
        else if (angle >= 13 && angle <= 39) {
            pos = position.RIGHT;
        }
        else {
            pos = position.NULL;
        }

        return pos;
    }

}
