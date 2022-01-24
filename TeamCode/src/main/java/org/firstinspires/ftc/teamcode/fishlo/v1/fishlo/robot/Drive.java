package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;

public class Drive extends SubSystem {

    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;

    ElapsedTime timer;

    Motor fl;
    Motor fr;
    Motor bl;
    Motor br;

    DcMotor arm;

    SampleMecanumDrive mecanumDrive = new SampleMecanumDrive(robot.hardwareMap);

    DcMotor[] motors = {frontLeft, frontRight, backLeft, backRight};

    MecanumDrive drive;

    private enum DriveControls {
        TANK,
        ARCADE,
        NONE
    }

    DriveControls[] driveControls = {DriveControls.TANK, DriveControls.ARCADE};
    DriveControls driveType;
    int driveIndex = 0;
    boolean telemetryEnabled;

    int cpr = 28;
    double gearRatio = 19.2;
    double diameter = DriveConstants.WHEEL_RADIUS * 2;
    double cpi = (cpr * gearRatio)/(Math.PI * diameter);
    double bias = 1.5;
    double meccyBias = 0.9;
    double conversion = cpi;
    int coeff = 1;

    boolean exit = false;
    /**
     * Construct a subsystem with the robot it applies to.
     *
     * @param robot
     */
    public Drive(Robot robot, boolean telemetryEnabled) {
        super(robot);
        this.telemetryEnabled = telemetryEnabled;
    }

    @Override
    public void init() {
        frontLeft = robot.hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = robot.hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = robot.hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = robot.hardwareMap.get(DcMotorEx.class, "backRight");
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm = robot.hardwareMap.dcMotor.get("arm");
        timer = new ElapsedTime();
        drive = new MecanumDrive(fl, fr, bl, br);

    }

    @Override
    public void handle() {

        double fl = 0, bl = 0, fr = 0, br = 0;
        driveType = driveControls[driveIndex];

        if (robot.gamepad1.dpad_up) {
            driveIndex++;
            if (driveIndex > 1) {
                driveIndex = 0;
            }
        }

        switch (driveType) {
            case ARCADE:
                double y = robot.gamepad1.left_stick_y; // Remember, this is reversed!
                double x = -robot.gamepad1.left_stick_x; // Counteract imperfect strafing
                double rx = -robot.gamepad1.right_stick_x;

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio, but only when
                // at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double frontLeftPower = (y + x + rx) / denominator;
                double backLeftPower = (y - x + rx) / denominator;
                double frontRightPower = (y - x - rx) / denominator;
                double backRightPower = (y + x - rx) / denominator;

                fl = frontLeftPower;
                bl = backLeftPower;
                fr = frontRightPower;
                br = backRightPower;

                break;

            case TANK:
                fl = robot.gamepad1.left_stick_y / coeff;
                fr = robot.gamepad1.right_stick_y / coeff;
                bl = robot.gamepad1.left_stick_y / coeff;
                br = robot.gamepad1.right_stick_y / coeff;
                if (robot.gamepad1.right_bumper) {
                    fl = -1 / coeff;
                    fr = 1 / coeff;
                    bl = 1 / coeff;
                    br = -1 / coeff;
                }
                if (robot.gamepad1.left_bumper) {
                    fl = 1 / coeff;
                    fr = -1 / coeff;
                    bl = -1 / coeff;
                    br = 1  / coeff;
                }
                if (robot.gamepad1.right_trigger >= 0.5) {
                    coeff = 2;
                }
                if  (robot.gamepad1.left_trigger >= 0.5) {
                    coeff = 1;
                }
                break;
        }

        if (robot.gamepad1.left_trigger >= 0.5) {
            coeff = 2;
        }
        else {
            coeff = 1;
        }

        drive(fl, bl, fr, br);
        if (telemetryEnabled) {
            robot.telemetry.addData("Drive - Dat - Drive Controls", driveType.name());
            robot.telemetry.addLine("Drive - Dat - Motors")
                    .addData("frontLeft", fl)
                    .addData("backLeft", bl)
                    .addData("frontRight", fr)
                    .addData("backRight", br);
            robot.telemetry.addLine("Drive - Dat - Inputs")
                    .addData("LeftY", -robot.gamepad1.left_stick_y)
                    .addData("RightY", -robot.gamepad1.right_stick_y)
                    .addData("LeftX", robot.gamepad1.left_stick_x)
                    .addData("RightX", robot.gamepad1.right_stick_x);
            robot.telemetry.addData("arm pos", arm.getCurrentPosition());
        }
    }

    public void strafe(double inches, double power, boolean timerOn, double time)
    {
        //
        resetEncoders();
        int move = -(int)(Math.round(inches*conversion));
        //
        backLeft.setTargetPosition(backLeft.getCurrentPosition() - move);
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + move);
        backRight.setTargetPosition(backRight.getCurrentPosition() + move);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() - move);
        //
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);
        backLeft.setPower(power);
        //
        while (frontLeft.isBusy()  && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            if(timerOn && timer.seconds() <= time) return;
        }
        stop();
    }


    @Override
    public void stop()
    {
        left(0);
        right(0);
    }

    public void turn(double angle)
    {
        mecanumDrive.turn(Math.toRadians(angle));
    }

    public void driveLeftTime(double time, double power) {
        while (timer.seconds() <= time) {
            frontLeft.setPower(power);
            frontRight.setPower(power);
        }
        left(0);
    }


    public void left(double power){
        frontLeft.setPower(power);
        backLeft.setPower(power);
    }
    public void right(double power){
        frontRight.setPower(power);
        backRight.setPower(power);
    }
    public void drive(double inches, double power, boolean timerOn, double time)
    {
        //
        resetEncoders();
        int move = -(int)(Math.round(inches*conversion));
        //
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + move);
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + move);
        backRight.setTargetPosition(backRight.getCurrentPosition() + move);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + move);
        //
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);
        backLeft.setPower(power);
        //
        robot.telemetry.addLine("enter while loop");
        robot.telemetry.update();
        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            if(timerOn && timer.seconds() <= time) return;
        }
        stop();
    }

    public void driveleft(double inches, double power, boolean timerOn, double time)
    {
        //
        resetEncoders();
        int move = -(int)(Math.round(inches*conversion));
        //
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + move);
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + move);
       // backRight.setTargetPosition(backRight.getCurrentPosition() + move);
       // frontRight.setTargetPosition(frontRight.getCurrentPosition() + move);
        //
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        frontLeft.setPower(power);
       // frontRight.setPower(power);
      //  backRight.setPower(power);
        backLeft.setPower(power);
        //
        robot.telemetry.addLine("enter while loop");
        robot.telemetry.update();
        timer.reset();
        while (frontLeft.isBusy() && backLeft.isBusy()) {
            if(timerOn && timer.seconds() >= time) return;
        }
        stop();
    }

    public void driveRight(double inches, double power, boolean timerOn, double time) {
        //
        resetEncoders();
        int move = -(int)(Math.round(inches*conversion));
        //
//        backLeft.setTargetPosition(backLeft.getCurrentPosition() + move);
//        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + move);
        backRight.setTargetPosition(backRight.getCurrentPosition() + move);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + move);
        //
//        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
//        frontLeft.setPower(power);
         frontRight.setPower(power);
          backRight.setPower(power);
//        backLeft.setPower(power);
        //
        robot.telemetry.addLine("enter while loop");
        robot.telemetry.update();
        timer.reset();
        while (frontRight.isBusy() && backRight.isBusy()) {
            if(timerOn && timer.seconds() >= time) return;
        }
        stop();
    }


    public enum Direction {
        BACK_LEFT,
        BACK_RIGHT,
        FRONT_RIGHT,
        FRONT_LEFT
    }

    public void drive(double fl, double bl, double fr, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    public void resetEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public double getbackrightpower() {
        return backRight.getPower();
    }
}