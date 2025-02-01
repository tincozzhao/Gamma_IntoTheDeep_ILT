package org.firstinspires.ftc.teamcode.opmode.OpMode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.hardware.BotCoefficients;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

//
@TeleOp(name = "OpMode3")

public class OpMode3 extends LinearOpMode {
    public ElapsedTime mRunTime = new ElapsedTime();

    RobotHardware robot = new RobotHardware();

    private int sleepMs1 = 0;
    private boolean bMoveUpSlider = false;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        // Put initialization blocks here.
        int liftBasePosition0 = robot.liftArm.getCurrentPosition();

        waitForStart();
        while (opModeIsActive()) {
            double horizontal = -1.0 * gamepad1.right_stick_x * 0.6;
            double vertical = gamepad1.right_stick_y * 0.6;
            double turn = -1.0 * gamepad1.left_stick_x * 0.6;

            double flPower = vertical + turn + horizontal;
            double frPower = vertical - turn - horizontal;
            double blPower = vertical + turn - horizontal;
            double brPower = vertical - turn + horizontal;
            double scaling = Math.max(1.0,
                    Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)),
                            Math.max(Math.abs(blPower), Math.abs(brPower))));
            flPower = flPower / scaling;
            frPower = frPower / scaling;
            blPower = blPower / scaling;
            brPower = brPower / scaling;
            robot.setDrivePower(flPower, frPower, blPower, brPower);

            // robot.setDrivePower(vertical + turn + horizontal, vertical
            // - turn - horizontal, vertical + turn - horizontal, vertical - turn + horizontal);

            telemetry.addLine(String.format("FL: %d \nBL %d \nFR: %d \nBR: %d ",
                    robot.motorfl.getCurrentPosition(),
                    robot.motorbl.getCurrentPosition(),
                    robot.motorfr.getCurrentPosition(),
                    robot.motorbr.getCurrentPosition()
            ));


//make sure one of the directions is correct/reversed
            //misumi slide start
            // To use continuous servo:
            // 1) change to continuous  rotation servo by using servo programmer
            // 2) on driver station, configured it as continuous rotation servo
            // 3) in Java code, use class "CRServo"
            if (gamepad2.left_stick_y > 0.7) { //if joystick moved up
                //misumi slide extends
                robot.misumiSlide.setPower(-1.0);
            }
            else if(gamepad2.left_stick_y < -0.7) {// if joystick moves down
                // misumi slide retract
                robot.misumiSlide.setPower(1.0);
            }
            else { //stop
                robot.misumiSlide.setPower(0.0);
            }


//linear slide
            if (gamepad2.right_stick_y < -0.7) { //when sticky y is on, going up
//                robot.liftArm.setTargetPosition(500);
//                robot.liftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.liftArm.setPower(BotCoefficients.SLIDER_UP_POWER);
                bMoveUpSlider = true;
            }
            else if (gamepad2.right_stick_y > 0.7) { //if sticky y is on, go down
               robot.liftArm.setPower(BotCoefficients.SLIDER_DOWN_POWER);
               bMoveUpSlider = false;
            }
            else {
                if (bMoveUpSlider) { // if last slider action is going up, hold slider
                    robot.liftArm.setPower(BotCoefficients.SLIDER_HOLD_POWER);
                }
                else { // otherwise, no needs to hold slider power.
                    robot.liftArm.setPower(0);
                }
            }

//grabberX on horizontal (misumi slide)
            if (gamepad2.left_trigger > 0.5) {
                robot.grabberX.setPosition(0.4); // open
            }
            else if (gamepad2.left_bumper) {
                robot.grabberX.setPosition(0.65); // close
            }
            else {
              //  robot.grabberX.setPosition(0);
            }

            if (gamepad2.dpad_up) {
                robot.grabberXtilt.setPosition(1); // tilt up grabber
            }
            else if (gamepad2.dpad_down) {
                robot.grabberXtilt.setPosition(0.1); // tilt down grabber
            }
            else {
                //  robot.grabberX.setPosition(0);
            }


//grabberY (linearslide)
            if (gamepad2.right_trigger > 0.5) {
                robot.grabberY.setPosition(0.4); // open
           } else if (gamepad2.right_bumper) {
                robot.grabberY.setPosition(0.8); // close
            } else{
               // robot.grabberY.setPosition(0.0);
            }

//tilt servo on Y vertical slide
            if (gamepad2.y) {
                robot.grabberYtilt.setPosition(0.2);    // tilt up

            } else if (gamepad2.a) {
                robot.grabberYtilt.setPosition(0.75);      // tilt down
                // AND open the grabber

            } else {
                // robot.grabberYtilt.setPosition(0);
            }

//tilt servo #2
            /*
            if (gamepad2.left_stick_y > 0.7) {
                robot.grabberYtilt.setPosition(0.0);

            } else if (gamepad2.left_stick_y < -0.7) {
                robot.grabberYtilt.setPosition(0.6);
            }

*/
        }


        //emergency releases
    }


    void liftHexArm(int ticks, double power, long timeOutMills) {
        long timeCurrent, timeBegin;
        timeBegin = timeCurrent = System.currentTimeMillis();
        {

        }



     /*

        robot.liftHex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftHex.setTargetPosition(ticks);
        robot.liftHex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftHex.setPower(power);
        while(opModeIsActive()
                && robot.liftHex.isBusy()
                && (timeCurrent - timeBegin) < timeOutMills) {
            timeCurrent = System.currentTimeMillis();





    private void TiltLiftOne ( double crankPowerBegin, int crankTimeMs, double crankPowerEnd,
        double liftPowerBegin, int liftTimeMs, double liftPowerEnd){
            //tilt the lift to be upright
            robot.liftHex.setPower(crankPowerBegin);   //set motor power
            sleep(crankTimeMs);          // let motor run for some time seconds.
            robot.liftHex.setPower(crankPowerEnd);   //set lower motor power to maintain the position

            // Extend liftArm
            robot.liftArm.setPower(liftPowerBegin);
            sleep(liftTimeMs);             // let motor run for some time seconds.
            robot.liftArm.setPower(liftPowerEnd);

        }

        }


      */

        }
    }