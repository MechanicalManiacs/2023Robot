package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;

public class Drive extends SubSystem {

    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;

    //28 * 20 / (2ppi * 4.125)
    Double gearratio = 19.2;
    Double width = 12.5; //inches
    Integer cpr = 28; //counts per rotation
    Double diameter = 4.125;
    Double cpi = (cpr * gearratio) / (Math.PI * diameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))
    Double bias = 0.8;//default 0.8
    Double meccyBias = 0.9;//change to adjust only strafing movement
    //
    Double conversion = cpi * bias;
    Boolean exit = false;

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    ElapsedTime timer;

    Motor fl;
    Motor fr;
    Motor bl;
    Motor br;

    DcMotor arm;

    SampleMecanumDrive mecanumDrive;

    DcMotor[] motors = {frontLeft, frontRight, backLeft, backRight};

    MecanumDrive drive;

    public enum DriveControls {
        TANK,
        ARCADE,
        NONE
    }

    DriveControls[] driveControls = {DriveControls.TANK, DriveControls.ARCADE};
    public static DriveControls driveType;
    int driveIndex = 0;
    boolean telemetryEnabled;

    int coeff = 1;

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
        mecanumDrive = new SampleMecanumDrive(robot.hardwareMap);
        frontLeft = robot.hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = robot.hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = robot.hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = robot.hardwareMap.get(DcMotorEx.class, "backRight");
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        arm = robot.hardwareMap.dcMotor.get("arm");
        timer = new ElapsedTime();
        drive = new MecanumDrive(fl, fr, bl, br);

    }

    @Override
    public void handle() {

        double fl = 0, bl = 0, fr = 0, br = 0;
        driveType = driveControls[driveIndex];

        if (robot.gamepad1.a) {
            driveIndex++;
            if (driveIndex > 1) {
                driveIndex = 0;
            }
        }

        if (robot.gamepad1.right_trigger >= 0.5) {
            Intake.driveOuttake();
        }

        switch (driveType) {
            case ARCADE:
                double y = -robot.gamepad1.left_stick_y; // Remember, this is reversed!
                double x = robot.gamepad1.left_stick_x; // Counteract imperfect strafing
                double rx = robot.gamepad1.right_stick_x;

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio, but only when
                // at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double frontLeftPower = (y + x + rx) / denominator;
                double backLeftPower = (y - x + rx) / denominator;
                double frontRightPower = (y - x - rx) / denominator;
                double backRightPower = (y + x - rx) / denominator;

                fl = frontLeftPower / coeff;
                bl = backLeftPower / coeff;
                fr = frontRightPower / coeff;
                br = backRightPower / coeff;
                break;

            case TANK:
                fl = -robot.gamepad1.left_stick_y / coeff;
                fr = -robot.gamepad1.right_stick_y / coeff;
                bl = -robot.gamepad1.left_stick_y / coeff;
                br = -robot.gamepad1.right_stick_y / coeff;
                if (robot.gamepad1.left_bumper) {
                    fl = -1 / coeff;
                    fr = 1 / coeff;
                    bl = 1 / coeff;
                    br = -1 / coeff;
                }
                if (robot.gamepad1.right_bumper) {
                    fl = 1 / coeff;
                    fr = -1 / coeff;
                    bl = -1 / coeff;
                    br = 1 / coeff;
                }
                if (robot.gamepad1.left_trigger >= 0.5) {
                    coeff = 2;
                } else {
                    coeff = 1;
                }
                break;
        }

        if (robot.gamepad1.left_trigger >= 0.5) {
            coeff = 2;
        } else {
            coeff = 1;
        }

        drive(fl, bl, fr, br);
        if (telemetryEnabled) {
            robot.telemetry.addData("Drive - Dat - Drive Controls", driveType.name());
//            robot.telemetry.addLine("Drive - Dat - Motors")
//                    .addData("frontLeft", fl)
//                    .addData("backLeft", bl)
//                    .addData("frontRight", fr)
//                    .addData("backRight", br);
//            robot.telemetry.addLine("Drive - Dat - Inputs")
//                    .addData("LeftY", -robot.gamepad1.left_stick_y)
//                    .addData("RightY", -robot.gamepad1.right_stick_y)
//                    .addData("LeftX", robot.gamepad1.left_stick_x)
//                    .addData("RightX", robot.gamepad1.right_stick_x);
//            robot.telemetry.addData("arm pos", arm.getCurrentPosition());
        }
    }

    public void strafe(double inches, double speed, boolean timerOn, double time) {
        //
        resetEncoders();
        //
        int move = -(int)(Math.round(inches * cpi * meccyBias));
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
        frontLeft.setPower(speed);
        backLeft.setPower(speed);
        frontRight.setPower(speed);
        backRight.setPower(speed);
        //
        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()){}
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
        return;
    }


    @Override
    public void stop() {
        left(0);
        right(0);
    }

    public void turn(double angle) {
        mecanumDrive.turn(Math.toRadians(angle));
    }

    public void driveLeftTime(double time, double power) {
        while (timer.seconds() <= time) {
            frontLeft.setPower(power);
            frontRight.setPower(power);
        }
        left(0);
    }


    public void left(double power) {
        frontLeft.setPower(power);
        backLeft.setPower(power);
    }

    public void right(double power) {
        frontRight.setPower(power);
        backRight.setPower(power);
    }

    public void drive(double inches, double speed, boolean timerOn, double time){
        resetEncoders();
        //
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
        frontLeft.setPower(speed);
        backLeft.setPower(speed);
        frontRight.setPower(speed);
        backRight.setPower(speed);
        //eg
        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()){
            if (exit){
                frontRight.setPower(0);
                frontLeft.setPower(0);
                backRight.setPower(0);
                backLeft.setPower(0);
                return;
            }
        }
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
        return;
    }

    public void driveStall(double power) {
        while (!frontLeft.isOverCurrent() && !frontRight.isOverCurrent() && !backRight.isOverCurrent() && !backLeft.isOverCurrent()) {
            frontRight.setPower(power);
            frontLeft.setPower(power);
            backRight.setPower(power);
            backLeft.setPower(power);
        }
        stop();
    }

    public void driveleft(double inches, double power, boolean timerOn, double time) {
        //
        resetEncoders();
        int move = -(int) (Math.round(inches * conversion));
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
        timer.reset();
        while (frontLeft.isBusy() && backLeft.isBusy()) {
            if (timerOn && timer.seconds() >= time) return;
        }
        stop();
    }

    public void driveRight(double inches, double power, boolean timerOn, double time) {
        //
        resetEncoders();
        int move = -(int) (Math.round(inches * conversion));
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
        timer.reset();
        while (frontRight.isBusy() && backRight.isBusy()) {
            if (timerOn && timer.seconds() >= time) return;
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

    public double getbackRightpower() {
        return backRight.getPower();
    }

    public void turnWithGyro(double degrees, double speedDirection) {
        //<editor-fold desc="Initialize">
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double yaw = -angles.firstAngle;//make this negative
        robot.telemetry.addData("Speed Direction", speedDirection);
        robot.telemetry.addData("Yaw", yaw);
        robot.telemetry.update();
        //
        robot.telemetry.addData("stuff", speedDirection);
        robot.telemetry.update();
        //
        double first;
        double second;
        //</editor-fold>
        //
        if (speedDirection > 0) {//set target positions
            //<editor-fold desc="turn right">
            if (degrees > 10) {
                first = (degrees - 10) + devertify(yaw);
                second = degrees + devertify(yaw);
            } else {
                first = devertify(yaw);
                second = degrees + devertify(yaw);
            }
            //</editor-fold>
        } else {
            //<editor-fold desc="turn left">
            if (degrees > 10) {
                first = devertify(-(degrees - 10) + devertify(yaw));
                second = devertify(-degrees + devertify(yaw));
            } else {
                first = devertify(yaw);
                second = devertify(-degrees + devertify(yaw));
            }
            //
            //</editor-fold>
        }
        //
        //<editor-fold desc="Go to position">
        Double firsta = convertify(first - 5);//175
        Double firstb = convertify(first + 5);//-175
        //
        turnWithEncoder(speedDirection);
        //
        if (Math.abs(firsta - firstb) < 11) {
            while (!(firsta < yaw && yaw < firstb)) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                robot.telemetry.addData("Position", yaw);
                robot.telemetry.addData("first before", first);
                robot.telemetry.addData("first after", convertify(first));
                robot.telemetry.update();
            }
        } else {
            //
            while (!((firsta < yaw && yaw < 180) || (-180 < yaw && yaw < firstb))) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                robot.telemetry.addData("Position", yaw);
                robot.telemetry.addData("first before", first);
                robot.telemetry.addData("first after", convertify(first));
                robot.telemetry.update();
            }
        }
        //
        Double seconda = convertify(second - 5);//175
        Double secondb = convertify(second + 5);//-175
        //
        turnWithEncoder(speedDirection / 3);
        //
        if (Math.abs(seconda - secondb) < 11) {
            while (!(seconda < yaw && yaw < secondb)) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                robot.telemetry.addData("Position", yaw);
                robot.telemetry.addData("second before", second);
                robot.telemetry.addData("second after", convertify(second));
                robot.telemetry.update();
            }
            while (!((seconda < yaw && yaw < 180) || (-180 < yaw && yaw < secondb))) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                robot.telemetry.addData("Position", yaw);
                robot.telemetry.addData("second before", second);
                robot.telemetry.addData("second after", convertify(second));
                robot.telemetry.update();
            }
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }
        //</editor-fold>
        //
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void initGyro() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "GyroCal.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        //
        imu = robot.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void turnWithEncoder(double input) {
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //
        frontLeft.setPower(-input);
        backLeft.setPower(-input);
        frontRight.setPower(input);
        backRight.setPower(input);
    }

    public double devertify(double degrees) {
        if (degrees < 0) {
            degrees = degrees + 360;
        }
        return degrees;
    }

    public double convertify(double degrees) {
        if (degrees > 179) {
            degrees = -(360 - degrees);
        } else if (degrees < -180) {
            degrees = 360 + degrees;
        } else if (degrees > 360) {
            degrees = degrees - 360;
        }
        return degrees;
    }
}