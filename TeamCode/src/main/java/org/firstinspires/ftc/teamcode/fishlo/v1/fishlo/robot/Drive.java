package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;

public class Drive extends SubSystem {

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    SampleMecanumDrive mecanumDrive = new SampleMecanumDrive(robot.hardwareMap);

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
        frontLeft = robot.hardwareMap.dcMotor.get("frontLeft");
        frontRight = robot.hardwareMap.dcMotor.get("fronRight");
        backLeft = robot.hardwareMap.dcMotor.get("backLeft");
        backRight = robot.hardwareMap.dcMotor.get("backRight");

        robot.telemetry.addLine("Motors initialized");
    }

    @Override
    public void handle() {
        double driveSpeed = -robot.gamepad1.left_stick_y / 3;
        double rightY = robot.gamepad1.right_stick_y / 3;
        double turnSpeed = 0;
        double strafeSpeed = robot.gamepad1.left_stick_x / 3;


        if (Math.abs(robot.gamepad1.right_stick_x) > 0.1) {
            turnSpeed = robot.gamepad1.right_stick_x / 3;
        }

        driveIndex = 1;

        driveType = driveControls[driveIndex];
        runDrive(driveType, driveSpeed, strafeSpeed, turnSpeed, rightY, -driveSpeed);

        robot.telemetry.addData("Drive - Dat - Drive Controls", driveType.name());
        robot.telemetry.addData("Drive - Dat - Drive Speed", driveSpeed);
        robot.telemetry.addData("Drive - Dat - Turn Speed", turnSpeed);
        robot.telemetry.addData("Drive - Dat - GamepadX", robot.gamepad1.left_stick_x);
        robot.telemetry.addData("Drive - Dat - Strafe Speed", strafeSpeed);
        robot.telemetry.addData("Drive - Set - frontLeft", frontLeft.getPower());
        robot.telemetry.addData("Drive - Set - backLeft", backLeft.getPower());
        robot.telemetry.addData("Drive - Set - frontRight", frontRight.getPower());
        robot.telemetry.addData("Drive - Set - backRight", backRight.getPower());
    }

    @Override
    public void stop() {

    }

    public void runDrive(DriveControls driveType, double driveSpeed, double strafeSpeed, double turnSpeed, double rightY, double leftY) {
        switch (driveType) {
            case ARCADE:
                mecanumDrive.setWeightedDrivePower(
                        new Pose2d(
                                driveSpeed,
                                -strafeSpeed,
                                -turnSpeed
                        )
                );
                break;

            case TANK:
                if (robot.gamepad1.right_bumper) {
                    if (robot.gamepad1.right_trigger < 0.5) {
                        strafe(0.5);
                    }
                    else if (robot.gamepad1.left_trigger < 0.5) {
                        strafe(0.3);
                    }
                    else {
                        strafe(1);
                    }
                }
                else if (robot.gamepad1.left_bumper) {
                    if (robot.gamepad1.right_trigger < 0.5) {
                        strafe(-0.5);
                    }
                    else if (robot.gamepad1.left_trigger < 0.5) {
                        strafe(-0.3);
                    }
                    else {
                        strafe(-1);
                    }
                }
                else {
                    left(-leftY);
                    right(-rightY);
                }
                break;

            default:
                driveType = DriveControls.TANK;
        }
    }

    private void left(double power) {
        try {
            frontLeft.setPower(power);
            backLeft.setPower(power);
        } catch(Exception ex) {}
    }

    private void right(double power) {
        try {
            frontRight.setPower(power);
            backRight.setPower(power);
        } catch(Exception ex) {}
    }

    public void drive(double leftPower, double rightPower) {
        left(leftPower);
        right(rightPower);
    }

    public void strafe(double power) {
        frontLeft.setPower(power);
        backLeft.setPower(-power);
        frontRight.setPower(-power);
        backRight.setPower(power);
    }

    public void turn (double power) {

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(-power);
        backRight.setPower(-power);
    }

}