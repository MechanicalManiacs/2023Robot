package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;

public class Intake extends SubSystem {

    DcMotor arm;
    DcMotor intake;
    DcMotor duck;

    ElapsedTime timer = new ElapsedTime();

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
    }

    @Override
    public void handle() {
        arm.setPower(robot.gamepad2.right_stick_y);
    }

    public enum IntakeState {
        ON,
        OFF,
        REVERSE
    }

    public void armToLevel(int level) {
        double ticksPerRev = 537.7;
        if (level == 0) {
            int target = (int) (ticksPerRev * (0.4));
            arm.setTargetPosition(target);
        }
        else if (level == 1) {
            int target = (int) (ticksPerRev * (0.8));
            arm.setTargetPosition(target);
        }
        else if (level == 2) {
            int target = (int) (ticksPerRev * (1.2));
            arm.setTargetPosition(target);
        }

        arm.setPower(0.5);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (arm.isBusy()) {
            //none
        }

        arm.setPower(0);
    }

    public void intake(IntakeState state) {
        if (state == IntakeState.OFF) {
            intake.setPower(0);
        }
        else if (state == IntakeState.ON) {
            intake.setPower(1);
        }
        else if (state == IntakeState.REVERSE) {
            intake.setPower(-0.5);
        }
    }

    public void spinCarousel(double power, double time){
        timer.reset();
        duck.setPower(power);
        if (timer.milliseconds() == time) {
            duck.setPower(0);
        }

    }
    @Override
    public void stop() {

    }
}
