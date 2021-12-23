package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;

public class Drive extends SubSystem {

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    SampleMecanumDrive mecanumDrive = new SampleMecanumDrive(robot.hardwareMap);

    private enum DriveControls {
        TANK,
        ARCADE
    }

    DriveControls[] driveControls = {DriveControls.TANK, DriveControls.ARCADE};
    DriveControls driveType;
    int driveIndex = 0;
    boolean telemetryEnabled;
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
        frontLeft = robot.hardwareMap.dcMotor.get("frontLeft");
        frontRight = robot.hardwareMap.dcMotor.get("frontRight");
        backLeft = robot.hardwareMap.dcMotor.get("backLeft");
        backRight = robot.hardwareMap.dcMotor.get("backRight");
    }

    @Override
    public void handle() {
        double y = -robot.gamepad1.left_stick_y;
        double x = robot.gamepad1.left_stick_x;
        double rx = robot.gamepad1.right_stick_x;
        double fl, bl, fr, br;
        double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        driveType = driveControls[driveIndex];

        switch (driveType) {
            case ARCADE:
                x = robot.gamepad1.left_stick_x * 1.1;
                fl = (y + x + rx) / denom;
                bl = (y - x + rx) / denom;
                fr = (y - x - rx) / denom;
                br = (y + x - rx) / denom;
                break;

            case TANK:
                fl = (y + x);
                bl = (y + x);
                fr = (y - x);
                br = (y - x);
                break;

            default:
                fl = 0;
                bl = 0;
                fr = 0;
                br = 0;
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
            robot.telemetry.addLine("Drive - Dat - InputData")
                    .addData("X", x)
                    .addData("Y", y)
                    .addData("RX", rx)
                    .addData("Denom", denom);
        }
    }

    public void strafe(double inches, double power)
    {
        double diameter = DriveConstants.WHEEL_RADIUS;
        double ticksPerRev = frontLeft.getMotorType().getTicksPerRev();

        //Rest Encoders.
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Calculate tick value that the motors need to turn.
        double circumference = 3.14*diameter;
        double rotationsNeeded = inches/circumference;
        int encoderDrivingTarget = (int)(rotationsNeeded*ticksPerRev);

        //Set tick value to the motors' target position.
        frontLeft.setTargetPosition(encoderDrivingTarget);
        frontRight.setTargetPosition(encoderDrivingTarget);
        backLeft.setTargetPosition(encoderDrivingTarget);
        backRight.setTargetPosition(encoderDrivingTarget);

        //Set the desired power
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(power);

        //Set the mode of the motors to run to the target position.
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy() )
        {
            //Do nothing until motors catch up.
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }


    @Override
    public void stop()
    {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }

    public void turn(double angle)
    {
        mecanumDrive.turn(Math.toRadians(angle));
    }

    public void drive(double inches, double power)
    {
        double diameter = DriveConstants.WHEEL_RADIUS;
        double ticksPerRev = frontLeft.getMotorType().getTicksPerRev();

        //Rest Encoders.
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Calculate tick value that the motors need to turn.
        double circumference = 3.14*diameter;
        double rotationsNeeded = inches/circumference;
        int encoderDrivingTarget = (int)(rotationsNeeded*ticksPerRev);

        //Set tick value to the motors' target position.
        frontLeft.setTargetPosition(encoderDrivingTarget);
        frontRight.setTargetPosition(encoderDrivingTarget);
        backLeft.setTargetPosition(encoderDrivingTarget);
        backRight.setTargetPosition(encoderDrivingTarget);

        //Set the desired power
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        //Set the mode of the motors to run to the target position.
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy() )
        {
            //Do nothing until motors catch up.
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }

    public enum Direction {
        BACK_LEFT,
        BACK_RIGHT,
        FRONT_RIGHT,
        FRONT_LEFT
    }

    public void diagonal(double inches, Direction direction, double power)
    {
        double diameter = DriveConstants.WHEEL_RADIUS;
        double ticksPerRev = frontLeft.getMotorType().getTicksPerRev();

        //Rest Encoders.
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Calculate tick value that the motors need to turn.
        double circumference = 3.14*diameter;
        double rotationsNeeded = inches/circumference;
        int encoderDrivingTarget = (int)(rotationsNeeded*ticksPerRev);

        //Set tick value to the motors' target position.
        frontLeft.setTargetPosition(encoderDrivingTarget);
        frontRight.setTargetPosition(encoderDrivingTarget);
        backLeft.setTargetPosition(encoderDrivingTarget);
        backRight.setTargetPosition(encoderDrivingTarget);

        //Set the desired power
        switch (direction) {
            case BACK_LEFT:
                frontLeft.setPower(-power);
                backRight.setPower(-power);
                break;
            case BACK_RIGHT:
                backLeft.setPower(-power);
                frontRight.setPower(-power);
                break;
            case FRONT_LEFT:
                backLeft.setPower(power);
                frontRight.setPower(power);
                break;
            case FRONT_RIGHT:
                frontLeft.setPower(power);
                backRight.setPower(power);
        }

        //Set the mode of the motors to run to the target position.
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy() )
        {
            //Do nothing until motors catch up.
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void drive(double fl, double bl, double fr, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    };
}