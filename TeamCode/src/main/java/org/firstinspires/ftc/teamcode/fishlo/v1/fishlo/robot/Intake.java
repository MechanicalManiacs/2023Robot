package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot;

import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.arcrobotics.ftclib.hardware.SensorDistance;
import com.arcrobotics.ftclib.hardware.SensorDistanceEx;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.sun.tools.javac.tree.DCTree;
import com.sun.tools.javac.util.Position;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;

import java.time.OffsetDateTime;

public class Intake extends SubSystem {

    public DcMotor arm;
    public static DcMotor intake;
    public DcMotor duck;

    public DistanceSensor distance;

    int count = 0;

    public CRServo capstoneClaw;

    double coeff = 1;

    ElapsedTime timer = new ElapsedTime();
    ElapsedTime duckTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    double[] position = {0, 0.5};
    int positionIndex = 0;

    int cpr = 28;
    double gearRatio = 19.2;
    double diameter = DriveConstants.WHEEL_RADIUS * 2;
    double cpi = (cpr * gearRatio)/(Math.PI * diameter);
    double bias = 0.8;
    double meccyBias = 0.9;
    double conversion = cpi * bias;

    /**
     * Construct a subsystem with the robot it applies to.
     *
     * @param robot
     */
    public Intake(Robot robot) {
        super(robot);
    }

    @Override
    public void init() {
        arm = robot.hardwareMap.dcMotor.get("arm");
        intake = robot.hardwareMap.dcMotor.get("intake");
        duck = robot.hardwareMap.dcMotor.get("carousel");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        capstoneClaw = robot.hardwareMap.crservo.get("capstoneClaw");
        distance = robot.hardwareMap.get(DistanceSensor.class, "distance");
    }

    public enum BlockIn {
        IN,
        NOT_IN
    }

    @Override
    public void handle() {
        capstoneClaw.setPower(-robot.gamepad2.right_stick_y/2);
        arm.setPower(Range.clip(-robot.gamepad2.left_stick_y, -0.9, 0.9));
        if (robot.gamepad2.dpad_up) {
            coeff = 1.5;
        }
        if (robot.gamepad2.dpad_down) {
            coeff = 1;
        }
        double increment = 0.1;
        if (robot.gamepad2.right_bumper) {
            duck.setPower(-0.8);
        }

        duck.setPower(robot.gamepad2.right_stick_x);

        if (robot.gamepad2.x) {
            duckTimer.reset();
            while (duckTimer.seconds() <= 0.7) {
                duck.setPower(-0.4);
            }

            duckTimer.reset();
            while (duckTimer.seconds() <= 1.5){
                duck.setPower(-0.8);
            }
            duck.setPower(0);
        }

        if (robot.gamepad2.right_trigger >= 0.5) {
            intake.setPower(1);
        }
        if (robot.gamepad2.left_trigger >= 0.5) {
            if (Drive.driveType == Drive.DriveControls.TANK) intake.setPower(-0.5);
            else if (Drive.driveType == Drive.DriveControls.ARCADE) intake(IntakeState.REVERSE);
        }
        else {
            intake(IntakeState.OFF);
        }
        if (robot.gamepad2.left_bumper) {
            intake.setPower(-0.4);
        }
        else {
            intake(IntakeState.OFF);
        }
    }

    public enum IntakeState {
        ON,
        OFF,
        REVERSE
    }

    public void armToLevel(int level, boolean manual, int pos) {
        double ticksPerRevHalf = 537.7 / 2;  //537.7
        double ticksPerRev = 537.7;
        int target = 0;
        if (!manual) {
            if (level == 0) {
                resetEncoder();
                target = 350;
            } else if (level == 1) {
                resetEncoder();
                target = 500;
            } else if (level == 2) {
                resetEncoder();
                target = 750;
            } else if (level == 3) {
                resetEncoder();
                target = 0;
            } else if (level == 4) {
                target = 900;
            }
        }
        else {
            target = pos;
        }
        arm.setTargetPosition(target);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.5);
        while (arm.isBusy()) {
            // do nohing
        }
    }

    public void intake(IntakeState state) {
        if (state == IntakeState.OFF) {
            intake.setPower(0);
        }
        else if (state == IntakeState.ON) {
            intake.setPower(1);
        }
        else if (state == IntakeState.REVERSE) {
            intake.setPower(-1);
        }
    }

    public static void driveOuttake() {
        intake.setPower(-0.6);
    }

    public void spinCarousel(String pos){
        if (pos.equals("Red")) {
            duckTimer.reset();
            while (duckTimer.seconds() <= 0.7) {
                duck.setPower(-0.4);
            }

            duckTimer.reset();
            while (duckTimer.seconds() <= 1.8){
                duck.setPower(-0.6);
            }
            duck.setPower(0);
        }
        else if (pos.equals("Blue")) {
            duckTimer.reset();
            while (duckTimer.seconds() <= 3) {
                duck.setPower(0.4);
            }
            duck.setPower(0);
        }

    }

    public double getMotorPos() {
        return arm.getCurrentPosition();
    }

    public void resetEncoder() {
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    @Override
    public void stop() {
        arm.setPower(0);
    }

    public BlockIn isBlockIn() {
        BlockIn blockIn = BlockIn.NOT_IN;
        if (distance.getDistance(DistanceUnit.INCH) <= 2.5) blockIn = BlockIn.IN;
        else blockIn = BlockIn.NOT_IN;
        return blockIn;
    }
}
