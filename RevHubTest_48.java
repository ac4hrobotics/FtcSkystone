/*
Copyright (c) 2016 Robert Atkinson
All rights reserved.
Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:
Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.
Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

//this test will have provided for thechanges on the lists. Anything that was deleted or commented out in this program can be found in Test 25.
@TeleOp(name="RevHubTest 48", group="Pushbot")
public class RevHubTest_48 extends OpMode {
    private enum LiftType  {
        Manual,
        Relic,
    }

    private int liftMax = 1500;

    private double maxSpeed = 1.0d;


    private SampleOp_EngineModule engine = new SampleOp_EngineModule();

    private ColorSensor colorRight = null;
    private ColorSensor colorLeft = null;


    private DcMotor relic_Motor = null;
    private DcMotor scissor_Relic = null;

    private int _scissPosition1 = 310;
    private int _scissPosition2 = 1237;

    private Servo gripLeft = null;
    private Servo gripRight = null;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        engine.Initialize(hardwareMap);


        relic_Motor = hardwareMap.get(DcMotor.class, "relicMotor");
        relic_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
       relic_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        scissor_Relic = hardwareMap.get(DcMotor.class, "scissor");
        scissor_Relic.setDirection(DcMotorSimple.Direction.REVERSE);
        scissor_Relic.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        gripLeft = hardwareMap.servo.get("gripLeft");
        gripLeft.setDirection(Servo.Direction.REVERSE);

        gripRight = hardwareMap.servo.get("gripRight");
        gripRight.setDirection(Servo.Direction.REVERSE);

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

       // relic_Motor.setTargetPosition(0);
        //scissor_Relic.setTargetPosition(0);
        relic_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        scissor_Relic.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        gripRight.setPosition(0.0d);
        gripLeft.setPosition(0.0d);

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double left = clamp(gamepad1.right_stick_y);
        double right = clamp(gamepad1.left_stick_y);


        //GAMEPAD 1 SECTION
        //For half speed and full speed toggle
        if (gamepad1.right_bumper) {
            engine.SetMaxMotorPower(0.5d);
        }
        else if (gamepad1.left_bumper){
            engine.SetMaxMotorPower(1.0d);
        }

        //This controls the mechanum movement staments as well as the joysticks. Please refer to sampleOp_Engine Module
        //For more information and how the joysticks and robot moves.
        if (!gamepad1.dpad_down && !gamepad1.dpad_left && !gamepad1.dpad_right && !gamepad1.dpad_up) {
            engine.SetSpeed(left, right);
        } else {
            engine.Move(GetInputs(gamepad1), 1.0d);
        }



        if (gamepad2.dpad_down  || gamepad2.y) {
            if (scissor_Relic.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                scissor_Relic.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            scissor_Relic.setPower(1.0d);
        }
        //For the Scissor Lift, the old value is 3220
        else if (gamepad2.dpad_up) {
            if (scissor_Relic.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                scissor_Relic.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            scissor_Relic.setPower(-1.0d);
        }
        else{
            scissor_Relic.setPower(0.0d);
        }




//
        if (gamepad2.dpad_right){
            if (relic_Motor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                relic_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            relic_Motor.setPower(1.0d);
        }
        else if (gamepad2.dpad_left || gamepad2.x){
            if (relic_Motor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                relic_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            relic_Motor.setPower(-1.0d);
        }
        else{
            relic_Motor.setPower(0.0d);
        }

        if (gamepad1.a) {
            gripLeft.setPosition(-0.2d);
            gripRight.setPosition(1.0d);
        }
        else if (gamepad1.b) {
            gripLeft.setPosition(0.3d);
            gripRight.setPosition(0.0d);
        }
        else if (gamepad1.x) {
            gripLeft.setPosition(0.6d);
            gripRight.setPosition(0.5d);
        }


        telemetry.addData("gripLeft Position: ", gripLeft.getPosition());
        telemetry.addData("gripRight Position: ", gripRight.getPosition());

        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */



    @Override
    public void stop() {
        engine.Stop();
        //lift_motor_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // lift_motor_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        relic_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        scissor_Relic.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public SampleOp_States.Dpad GetInputs(Gamepad gamepad) {
        if (gamepad.dpad_down && gamepad.dpad_left) {
            return SampleOp_States.Dpad.DownLeft;
        }
        else if (gamepad.dpad_down && gamepad.dpad_right) {
            return SampleOp_States.Dpad.DownRight;
        }
        else if (gamepad.dpad_down && gamepad.dpad_right) {
            return SampleOp_States.Dpad.DownRight;
        }
        else if (gamepad.dpad_up && gamepad.dpad_left) {
            return SampleOp_States.Dpad.UpLeft;
        }
        else if (gamepad.dpad_up && gamepad.dpad_right) {
            return SampleOp_States.Dpad.UpRight;
        }
        else if (gamepad.dpad_down) {
            return SampleOp_States.Dpad.Down;
        }
        else if (gamepad.dpad_left) {
            return SampleOp_States.Dpad.Left;
        }
        else if (gamepad.dpad_right) {
            return SampleOp_States.Dpad.Right;
        }
        else if (gamepad.dpad_up) {
            return SampleOp_States.Dpad.Up;
        }
        else {
            return SampleOp_States.Dpad.None;
        }
    }

    private double clamp(double value) {
        if (value > maxSpeed) {
            return maxSpeed;
        }
        else if (value < -maxSpeed) {
            return -maxSpeed;
        }
        else {
            return value;
        }
    }

    private int clamp(int min, int max, int value) {
        return Math.max(min, Math.min(max, value));
    }


    private String checkColor() {
        int left = colorLeft.red() - colorLeft.blue();
        int right = colorRight.red() - colorRight.blue();

        if (left > right) {
            return "LEFT";
        }
        else if (left < right) {
            return "RIGHT";
        }
        else {
            return "NONE";
        }
    }
}//Very NICE