package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.drivebase.HDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;

public class Drive extends SubSystem {

    private Motor frontLeft, frontRight, backLeft, backRight;

    HDrive xDrive;

    SampleMecanumDrive mecanumDrive = new SampleMecanumDrive(robot.hardwareMap);

    Servo claw;

    private enum DriveControls {
        TANK,
        ARCADE
    }

    DriveControls[] driveControls = {DriveControls.TANK, DriveControls.ARCADE};
    DriveControls driveType;
    int driveIndex = 0;
    /**
     * Construct a subsystem with the robot it applies to.
     *
     * @param robot
     */
    public Drive(Robot robot) {
        super(robot);
    }

    @Override
    public void init() {
        frontLeft = new Motor(robot.hardwareMap, "frontLeft");
        frontRight = new Motor(robot.hardwareMap, "frontRight");
        backLeft = new Motor(robot.hardwareMap, "backLeft");
        backRight = new Motor(robot.hardwareMap, "backRight");
        claw = robot.hardwareMap.servo.get("claw");
        xDrive = new HDrive(frontLeft, frontRight, backLeft, backRight);
    }

    @Override
    public void handle() {
        double driveSpeed = robot.gamepad1.left_stick_y / 4;
        double turnSpeed = 0;

        turnSpeed = robot.gamepad1.right_stick_x / 4;

        driveIndex = 1;

        driveType = driveControls[driveIndex];
        if (robot.gamepad1.y) {
            claw.setPosition(0.5);
        }
        if (robot.gamepad1.a) {
            claw.setPosition(0);
        }
        mecanumDrive.setWeightedDrivePower(new Pose2d(
                robot.gamepad1.left_stick_y,
                -robot.gamepad1.left_stick_x,
                -robot.gamepad1.right_stick_x
        ));

        robot.telemetry.addData("Drive - Dat - Drive Controls", driveType.name());
        robot.telemetry.addData("Drive - Dat - Drive Speed", driveSpeed);
        robot.telemetry.addData("Drive - Dat - Turn Speed", turnSpeed);
        robot.telemetry.addData("Drive - Dat - GamepadX", robot.gamepad1.left_stick_x);
    }

    @Override
    public void stop() {

    }

//    public void runDrive(DriveControls driveType) {
//        switch (driveType) {
//            case ARCADE:
//                xDrive.driveRobotCentric(
//                        gamepad1.getLeftX(),
//                        gamepad1.getLeftY(),
//                        gamepad1.getRightY()
//                );
//                break;
//
//            default:
//                driveType = DriveControls.ARCADE;
//        }
//    }

//    private void left(double power) {
//        try {
//            frontLeft.set(power);
//            backLeft.set(power);
//        } catch(Exception ex) {}
//    }
//
//    private void right(double power) {
//        try {
//            frontRight.set(power);
//            backRight.set(power);
//        } catch(Exception ex) {}
//    }
//
//    public void drive(double leftPower, double rightPower) {
//        left(leftPower);
//        right(rightPower);
//    }
//
//    public void strafe(double power) {
//        frontLeft.set(power);
//        backLeft.set(-power);
//        frontRight.set(-power);
//        backRight.set(power);
//    }
//
//    public void turn (double power) {
//        frontLeft.set(power);
//        backLeft.set(power);
//        frontRight.set(-power);
//        backRight.set(-power);
//    }

}