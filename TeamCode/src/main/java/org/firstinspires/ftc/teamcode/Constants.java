package org.firstinspires.ftc.teamcode;

public class Constants {


    public static class Pitch{
        public static final double PITCH_UP = .05,
                                   PITCH_LOW = .87;
    }

    public static class Braco{
        public static final double stagee0 = 0.0,  stage2 = 3.5, stage3 = 4.1,
                                   up_speed = 1,
                                   lower_speed = .9,
                                   adjust = .25;
        public static final int stage1 = 200, stage0 = 0;
     }

     public static class PitchBandeja{

        public static final double pitchStage1 = 0.0,
                                   pitchStage2 = 3.0,
                                   adjustPitch = .3;
     }

     public static class DriveMecanum{
        public static final double acceleration = 1,
                                          speed = 1;
     }

     public static class SistemaLinear{

        public static final int stage0 = 0;

     }

     public static class Pipeline{
         public static final double[][] AUTO_COLOR_LOW = {{50, 50, 30},  {90, 70, 70}},
                                       AUTO_COLOR_UP =  {{80, 180, 360}, {100, 180, 360}};

         public static final double[] TELE_COLOR_LOW = {75, 40, 40},
                                    TELE_COLOR_UP = {130, 230, 360};

         public static final int TOLERANCE_AREA = 1000;

    }
}
