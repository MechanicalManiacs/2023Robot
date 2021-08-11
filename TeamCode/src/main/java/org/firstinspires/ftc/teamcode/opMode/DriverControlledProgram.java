package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.Robot;

/**
 * A driver controlled program.
 * NOTE: This class will automatically stop itself two minutes after starting!
 * Usage:
 * This class should be extended by a single program in which you MUST:
 *  - Override #buildRobot()
 * and MAY:
 *  - Override #onStart()
 *  - Override #onUpdate()
 *  - Override #onStop()
 * @Author Jaxon Brown
 */
public abstract class DriverControlledProgram extends OpMode {
    public static Robot robot;
    /**
     * Build a robot. This should be overridden by your Program.
     * Construct your robot and make any necessary changes to the subsystems.
     * @return Robot this program controls.
     */
    protected abstract Robot buildRobot();

    /**
     * Called when the the program is started.
     */
    protected void onStart() {telemetry.setAutoClear(true);}

    /**
     * Called when the loop finishes.
     */
    protected void onUpdate() {}

    /**
     * Called when the robot is stopped.
     */
    protected void onStop() {}

    @Override
    public final void init() {


        robot = buildRobot();

        try {
            robot.init();
        } catch(Exception ex) {
            telemetry.addData("ERROR!!!", ex.getMessage());
        }
    }

    //    Timer timer;
    @Override
    public final void start() {
        onStart();
//        timer = new Timer();
//        timer.start();
    }

    @Override
    public final void loop() {
        robot.driverControlledUpdate();
        onUpdate();
    }

    @Override
    public final void stop() {
//        timer.stopThread();
        onStop();
    }

    /**
     * Gets the robot. If you properly cached your subcomponents in buildRobot(), you probably don't need this.
     * @return
     */
    protected final Robot getRobot() {
        return robot;
    }

}

//class Timer extends Thread {
//    private long stopTime;
//    private Robot robot;
//    private int soundIndex;
//    private int soundId;
//    private boolean exit;
//    private boolean soundPlaying;
//    private String  sounds[] =  {"ss_alarm", "ss_bb8_down", "ss_bb8_up", "ss_darth_vader", "ss_fly_by",
//            "ss_mf_fail", "ss_laser", "ss_laser_burst", "ss_light_saber", "ss_light_saber_long", "ss_light_saber_short",
//            "ss_light_speed", "ss_mine", "ss_power_up", "ss_r2d2_up", "ss_roger_roger", "ss_siren", "ss_wookie" };
//    Context myApp = robot.hardwareMap.appContext;
//    SoundPlayer.PlaySoundParams params;
//
//    public Timer() {
//        stopTime = System.currentTimeMillis() + 1000 * 121;
//        stopTime /= 1000;
//        robot = DriverControlledProgram.robot;
//        soundIndex = 0;
//        soundId = -1;
//        soundPlaying = false;
//
//        params = new SoundPlayer.PlaySoundParams();
//        params.loopControl = 0;
//        params.waitForNonLoopingSoundsToFinish = true;
//        exit = false;
//    }
//
//    public void run() {
//        while (!exit) {
//            robot.telemetry.addData("Time Remaining: ", stopTime);
//            stopTime -= System.currentTimeMillis() / 1000D;
//
//            if (stopTime < 60 && !soundPlaying) {
//                robot.telemetry.addLine("60 seconds Remaining!!!");
//                robot.telemetry.update();
//                soundPlaying = true;
//                SoundPlayer.getInstance().startPlaying(myApp, soundId, params, null,
//                        () -> {
//                            soundPlaying = false;
//                        });
//            }
//        }
//    }
//
//    public void stopThread() {
//        exit = true;
//    }
//}