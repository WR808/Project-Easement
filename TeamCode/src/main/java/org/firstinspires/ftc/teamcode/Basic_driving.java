/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Basic: OpMode", group = "Linear Opmode")

public class Basic_driving extends LinearOpMode {

    static final int CYCLE_MS = 50;     // period of each cycle
    final double pinchSpeed = 0.01;                   // sets rate to move servo
    double pinchOffset = 0;                       // Servo mid position
    double pinchVerticalPos = RoverBot.MIN_SERVO;
    double pinchHorizontalPos = RoverBot.MIN_SERVO;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    /* Constructor */
    private RoverBot robot = new RoverBot();


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.init(hardwareMap);
  //      sleep(5000);
        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  //      sleep(2000);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    //    sleep(2000);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    //    sleep(2000);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
     //   sleep(2000);

        robot.linearLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.pinchVertical.setDirection(Servo.Direction.FORWARD);
        robot.pinchHorizontal.setDirection(Servo.Direction.FORWARD);


      //  sleep(2000);

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            {

                // Setup a variable for each drive wheel to save power level for telemetry
                double leftFrontPower;
                double rightFrontPower;
                double leftBackPower;
                double rightBackPower;

                double liftPower;

                // choosing which button to use to move the linear slide
                double lift = gamepad2.right_stick_y;

                liftPower = Range.clip(lift, -5.0, 5.0);

                if (gamepad2.right_bumper)
                    pinchHorizontalPos += pinchSpeed;
                else if (gamepad2.left_bumper)
                    pinchHorizontalPos -= pinchSpeed;

                // Move both servos to new position.  Assume servos are mirror image of each other.
                pinchHorizontalPos = Range.clip(pinchHorizontalPos, robot.MIN_SERVO, robot.MAX_SERVO);
              //  robot.pinchHorizontal.setPosition(pinchHorizontalPos);

                if (gamepad2.a)
                    pinchVerticalPos += pinchSpeed;
                else if (gamepad2.b)
                    pinchVerticalPos -= pinchSpeed;

                // Move both servos to new position.  Assume servos are mirror image of each other.
                pinchVerticalPos = Range.clip(pinchVerticalPos, robot.MIN_SERVO, robot.MAX_SERVO);
              //  robot.pinchVertical.setPosition(pinchVerticalPos);
                //               sleep(CYCLE_MS);
                //               idle();
                // Choose to drive using either Tank Mode, or POV Mode
                // Comment out the method that's not used.  The default below is POV.

                // POV Mode uses left stick to go forward, and right stick to turn.
                // - This uses basic math to combine motions and is easier to drive straight.
                double drive = -gamepad1.left_stick_y;
                double turn = -gamepad1.right_stick_x;
                leftFrontPower = Range.clip(drive - turn, -1, 1.0);
                rightFrontPower = Range.clip(drive + turn, -1.0, 1.0);
                leftBackPower = Range.clip(drive - turn, -1.0, 1.0);
                rightBackPower = Range.clip(drive + turn, -1.0, 1.0);

                // Tank Mode uses one stick to control each wheel.
                // - This requires no math, but it is hard to drive forward slowly and keep straight.
                // leftPower  = -gamepad1.left_stick_y ;
                // rightPower = -gamepad1.right_stick_y ;

                // Send calculated power to wheels
                robot.leftFrontDrive.setPower(leftFrontPower);
                robot.rightFrontDrive.setPower(rightFrontPower);
                robot.leftBackDrive.setPower(leftBackPower);
                robot.rightBackDrive.setPower(rightBackPower);
                //               sleep(CYCLE_MS);
                //               idle();

                //send power to lift
                robot.linearLift.setPower(liftPower);
                //sleep(CYCLE_MS);
               // idle();
                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Motors", "leftfront (%.2f), rightfront (%.2f),leftback (%.2f),rightback (%.2f)", leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
                telemetry.addData("Pinch Horizontal Position ", "%2f", pinchHorizontalPos);
                telemetry.addData("Pinch Vertical Position ", "%2f", pinchVerticalPos);
                telemetry.addData("Linear Slide", "%2f", liftPower);
                telemetry.update();

            }
        }
    }

}