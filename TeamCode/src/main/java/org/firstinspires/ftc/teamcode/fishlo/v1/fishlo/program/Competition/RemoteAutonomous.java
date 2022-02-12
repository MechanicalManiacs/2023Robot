package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.Competition;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.program.FishloAutonomousProgram;
import org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Robot;

import java.util.ArrayList;

@Autonomous
public class RemoteAutonomous extends FishloAutonomousProgram {

    String placement;

    @Override
    protected Robot buildRobot() {
        return super.buildRobot();
    }

    @Override
    public void preMain() {
        telemetry.setAutoClear(true);
        intake.capstoneClaw.setPower(-0.2);
        sleep(100);
        intake.capstoneClaw.setPower(0);
        ArrayList<String> positions = new ArrayList<>(3);
        positions.add("Left");
        positions.add("Center");
        positions.add("Right");
        drive.initGyro();
        telemetry.addLine("Detecting Duck");
        telemetry.update();
        while (!isStarted()) {
            if (positions.contains(vision.getPlacement())) {
                placement = vision.getPlacement();
                telemetry.addData("Position", placement);
                telemetry.update();
            }
        }
    }

    @Override
    public void main() {
        placement = "Center";
        switch (placement) {
            case "Left":
                intake.armToLevel(0, false, 0);
                break;
            case "Center":
                intake.armToLevel(1, false, 0);
                break;
            case "Right":
                intake.armToLevel(2, false, 0);
                break;
        }
        //
        drive.drive(8, 0.4, false, 0);
        //
        drive.turnWithGyro(30, 0.4);
        //
        drive.drive(22, 0.4, false, 0);
        //
        intake.intake(Intake.IntakeState.REVERSE);
        sleep(1000);
        intake.intake(Intake.IntakeState.OFF);
        //
        sleep(200);
        //
//        drive.strafe(7, 0.4, false, 0);
        drive.turnWithGyro(50, 0.3);
        //
        intake.stop();
        intake.intake(Intake.IntakeState.ON);
        //
        drive.drive(23, 0.5, false, 0);
        //
        sleep(100);
        //
        intake.armToLevel(2, false, 0);
        //
        drive.turnWithGyro(140, -0.4);
        //
        sleep(200);
        //
        drive.drive(4, 0.4, false, 0);
        //
        intake.intake.setPower(-1);
        sleep(1000);
        intake.intake(Intake.IntakeState.OFF);
        //
        drive.turnWithGyro(30, -0.4);
        //
        intake.stop();
        intake.intake(Intake.IntakeState.ON);
        //
        drive.drive(24, 0.4, false, 0);
        //
        drive.driveleft(8, 0.4, false, 0);
        drive.driveRight(5, 0.4, false, 0);
        drive.driveleft(8, 0.4, false, 0);
        drive.driveRight(5, 0.4, false, 0);
        //
        intake.armToLevel(2, false, 0);
        //
        drive.turnWithGyro(170, 0.4);
        //
        drive.drive(24, 0.4, false, 0);
        //
        intake.intake.setPower(-1);
        sleep(1000);
        intake.intake(Intake.IntakeState.OFF);
        //
        drive.turnWithGyro(80, -0.4);
        //
        drive.strafe(-36, 0.4, false, 0);
        //
        drive.drive(-15, 0.4, false, 0);
        //
        intake.spinCarousel("Red");
        //
        drive.drive(-5, 0.4, false, 0);
        //
        intake.armToLevel(0, false, 0);
        //
        drive.drive(0.6, -0.6,-0.6, -0.6);
        sleep(7000);
        drive.stop();
        //
    }
}