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

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

//import com.qualcomm.robotcore.hardware.DistanceSensor;

//In this test I programmed everything according to the chart on my phone. Programmed the motors to move with encoders
@TeleOp(name="AutonomousTest_2", group="Pushbot")
public class AutonomousTest_2 extends OpMode {
    private SampleOp_EngineModule engine = new SampleOp_EngineModule();
    static SingularitySoundsSKYSTONE sounds = new SingularitySoundsSKYSTONE();
    //   private ColorSensor color_sensor;
    // private DistanceSensor distance_sensor = null;
    // private DcMotor encoder_motor = null;


    private double maxSpeed = 1.0d;
    private double moveSpeed = 1.0d;

    private int stage = 7;

    private double timeOffSet = 0;

    private int mechanum = 0;
    private int center = 1150;
    private int left = 1950;
    private int right = 350;

    private  String soundstring = sounds.sounds[0];
    SoundPlayer soundplaya = new SoundPlayer(3, 6);

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        engine.Initialize(hardwareMap);



        //accuator = hardwareMap.servo.get("accuator");


        engine.SetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


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


        engine.SetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sounds.runOpMode();
        mechanum = center;
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        mechanum = center;
        switch (stage) {
            case 7:
              //  SoundPlayer.getInstance().startPlaying(myApp, soundID, params, null,
                    //    new Runnable() {
                //            public void run() {
               //                 soundPlaying = false;
              //              }} );
                break;
            case 8:
                if (engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontLeft).getCurrentPosition() > -1000) {
                    if (engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontLeft).getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                        engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontLeft).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }

                    engine.SetSpeed(-0.3d, -0.3d);
                    //sounds.soundOn();

                    //  engine.SetSpeed(-0.20d, -0.20d);
                } else {
                    NextStage();
                }
                break;
            case 9:
                reset();
                NextStage();
                break;
            case 10:
                if (engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontLeft).getCurrentPosition() < 1150) {
                    if (engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontLeft).getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                        engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontLeft).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }

                    engine.Move(SampleOp_States.Dpad.Down, 0.30d);
                    //  engine.SetSpeed(-0.20d, -0.20d);
                } else {
                    NextStage();
                }
                break;
            case 11:
              reset();
              NextStage();
                break;
            case 12:
                if(engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontLeft).getCurrentPosition() < 1000){
                    if (engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontLeft).getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                        engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontLeft).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }

                    engine.SetSpeed(0.3d, 0.3d);
                }
                else{
                    NextStage();
                }
                break;
            case 13:
                reset();
                NextStage();
                break;
            case 14:
                if (engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontLeft).getCurrentPosition() > -1150) {
                    if (engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontLeft).getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                        engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontLeft).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
                    engine.Move(SampleOp_States.Dpad.Right, 0.30d);
                }

                NextStage();
                break;
            default:
                engine.Stop();
                break;
        }

        telemetry.addData("Stage: ", stage);
        telemetry.addData("FrontLeft: ", engine.GetPosition(SampleOp_EngineModule.EngineMotor.FrontLeft));
        telemetry.addData("FrontRight: ", engine.GetPosition(SampleOp_EngineModule.EngineMotor.FrontRight));
        telemetry.addData("BackLeft: ", engine.GetPosition(SampleOp_EngineModule.EngineMotor.BackLeft));
        telemetry.addData("BackRight: ", engine.GetPosition(SampleOp_EngineModule.EngineMotor.BackRight));

        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        engine.Stop();


    }

    public SampleOp_States.Dpad GetInputs(Gamepad gamepad) {
        if (gamepad.dpad_down && gamepad.dpad_left) {
            return SampleOp_States.Dpad.DownLeft;
        } else if (gamepad.dpad_down && gamepad.dpad_right) {
            return SampleOp_States.Dpad.DownRight;
        } else if (gamepad.dpad_down && gamepad.dpad_right) {
            return SampleOp_States.Dpad.DownRight;
        } else if (gamepad.dpad_up && gamepad.dpad_left) {
            return SampleOp_States.Dpad.UpLeft;
        } else if (gamepad.dpad_up && gamepad.dpad_right) {
            return SampleOp_States.Dpad.UpRight;
        } else if (gamepad.dpad_down) {
            return SampleOp_States.Dpad.Down;
        } else if (gamepad.dpad_left) {
            return SampleOp_States.Dpad.Left;
        } else if (gamepad.dpad_right) {
            return SampleOp_States.Dpad.Right;
        } else if (gamepad.dpad_up) {
            return SampleOp_States.Dpad.Up;
        } else {
            return SampleOp_States.Dpad.None;
        }
    }

    private double clamp(double value) {
        if (value > maxSpeed) {
            return maxSpeed;
        } else if (value < -maxSpeed) {
            return -maxSpeed;
        } else {
            return value;
        }
    }

    private void GoToStage(int target) {
        timeOffSet = time;
        stage = target;
    }

    private void NextStage() {
        timeOffSet = time;
        stage++;
    }

    private void PreviousStage() {
        timeOffSet = time;
        stage--;
    }


    private void turn(int degrees) {

    }

    private void move(int distance, double power) {
        if( engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontLeft).getCurrentPosition() > distance ){
        if (engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontLeft).getCurrentPosition() > distance) {
            if (engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontLeft).getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontLeft).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
            engine.SetSpeed(-power, -power);
        }
    }
    private void reset(){
        if (engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontLeft).getMode() != DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
            engine.SetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }


}