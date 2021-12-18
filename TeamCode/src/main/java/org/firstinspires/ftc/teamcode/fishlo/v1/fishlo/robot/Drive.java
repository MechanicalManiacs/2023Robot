package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.drivebase.HDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.AutomaticFeedforwardTuner;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;

public class Drive extends SubSystem {

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    HDrive xDrive;

    SampleMecanumDrive mecanumDrive = new SampleMecanumDrive(robot.hardwareMap);

    DcMotor claw;

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
        frontLeft = robot.hardwareMap.dcMotor.get("frontleft");
        frontRight = robot.hardwareMap.dcMotor.get("frontright");
        backLeft = robot.hardwareMap.dcMotor.get("backleft");
        backRight = robot.hardwareMap.dcMotor.get("backright");
        claw = robot.hardwareMap.dcMotor.get("claw");
    }

    @Override
    public void handle() {
        double driveSpeed = robot.gamepad1.left_stick_y / 4;
        double turnSpeed = 0;

        turnSpeed = robot.gamepad1.right_stick_x / 4;

        driveIndex = 1;

        driveType = driveControls[driveIndex];
        if (robot.gamepad1.y) {

        }
        if (robot.gamepad1.a) {

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

    public void strafe(double power)
    {
        frontLeft.setPower(power);
        backRight.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
    }

    @Override
    public void stop()
    {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }

    public void drive(double inches, double power)
    {
        int diameter = 1;
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
}