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

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import com.vuforia.CameraDevice;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfInt;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.ConceptVuMarkIdentification.JewelColor.NONE;

/**
 * This OpMode illustrates the basics of using the Vuforia engine to determine
 * the identity of Vuforia VuMarks encountered on the field. The code is structured as
 * a LinearOpMode. It shares much structure with {@link ConceptVuforiaNavigation}; we do not here
 * duplicate the core Vuforia documentation found there, but rather instead focus on the
 * differences between the use of Vuforia for navigation vs VuMark identification.
 *
 * @see ConceptVuforiaNavigation
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained in {@link ConceptVuforiaNavigation}.
 */

@SuppressWarnings({"unused", "DanglingJavadoc", "JavaDoc", "WeakerAccess", "ConstantConditions", "FieldCanBeLocal", "RedundantArrayCreation"})
@Autonomous(name="Concept: VuMark Id", group ="Concept")
//@Disabled
public class ConceptVuMarkIdentification extends LinearOpMode {

    public static final String TAG = "Vuforia VuMark Sample";

    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;

    public enum JewelColor {RED, BLUE, NONE}
    private JewelColor jewelColor = NONE;
    private static final double THRESHOLD = 0.4;
    private static final int BINS = 8;
    private static final float MIN_VALUE = 0.0f;
    private static final float MAX_VALUE = 255.0f;
    private JewelColor foundColor = JewelColor.NONE;

    private void initOpenCv()
    {
        BaseLoaderCallback mLoaderCallback =
                new BaseLoaderCallback(hardwareMap.appContext)
        {
            @Override
            public void onManagerConnected(int status)
            {
                super.onManagerConnected(status);
            }
        };

        if(!OpenCVLoader.initDebug())
        {
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_4_0,
                    hardwareMap.appContext,
                    mLoaderCallback);
        }
        else
        {
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
    }

    private void initVuforia()
    {
        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
        parameters.vuforiaLicenseKey =
                "AQgIvJ7/////AAAAGQSociXWO0kDvfP15zd4zOsS+fHJygDMLA" +
                        "1HhOJQ3FkeiPLGU6YW3ru+jzC6MGxM5tY1ajF4Y0plOpxhQGfS" +
                        "R4g3zFiP0IQavezWhGbjBCRMmYu8INy8KvoZ03crZe9wxxQJu9" +
                        "9KiNX3ZrbUevNXODKKzWyA9RqxxQHbJ3gpXoff4z1O9n211VOg" +
                        "EsJjrNZq8xJnznilyXwc8colJnZD/Adr6UmOzxoUGgaMrdPrlj" +
                        "McDJZU6uyoIrOjiv1G2r3iNjtd7LzKAANKrK/0IrO90MgRqQDr" +
                        "CAAJVHqqyyubMy8EqE5onzw/WFEcEwfQ6nolsNwYTEZb/JppU8" +
                        "9Q6DZmhz4FCT49shA+4PyNOzqsjhRC";
        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
    }

    Bitmap rgbImage = null;

    private Bitmap getImage()
    {
        VuforiaLocalizer.CloseableFrame frame;
        try
        {
            frame = vuforia.getFrameQueue().take();
        }
        catch (InterruptedException e)
        {
            RobotLog.ee(TAG, "InterruptedException getting frame");
            return null;
        }

        long numImages = frame.getNumImages();

        Image imgData = null;
        for(int i = 0; i < numImages; i++)
        {
            int format = frame.getImage(i).getFormat();
            if(format == PIXEL_FORMAT.RGB565)
            {
                imgData = frame.getImage(i);
                break;
            }
        }

        if(imgData != null)
        {
            int imgW = imgData.getWidth();
            int imgH = imgData.getHeight();

            Bitmap.Config imgT = Bitmap.Config.RGB_565;
            if(rgbImage == null) rgbImage = Bitmap.createBitmap(imgW, imgH, imgT);
            rgbImage.copyPixelsFromBuffer(imgData.getPixels());
        }
        frame.close();

        return rgbImage;
    }

    Mat showImg;
    Mat cvImg;
    Mat hsvImage;
    Mat histHue = new Mat();
    Mat histSat = new Mat();

    private JewelColor getJewelColor()
    {
        if(opModeIsActive() && jewelColor != NONE) return jewelColor;
        jewelColor = NONE;
        vuforia.setFrameQueueCapacity(1);

        double jTimeout = 1.0;
        ElapsedTime jtimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        while (opModeIsActive() && jewelColor == NONE && jtimer.seconds() < jTimeout)
        {
            Bitmap ballImg = getImage();
            if(ballImg == null) continue;

            int cvt = CvType.CV_8UC1;
            int inH = ballImg.getHeight();
            int inW = ballImg.getWidth();

            if(cvImg == null) cvImg = new Mat(inH, inW, cvt);

            nBitmapToMat2(rgbImage, cvImg.nativeObj, false);

            if(showImg == null) showImg = cvImg.clone();
            else cvImg.copyTo(showImg);

            if(hsvImage == null) hsvImage =
                    new Mat(showImg.width(), showImg.height(), showImg.type());

            Imgproc.cvtColor(showImg, hsvImage, Imgproc.COLOR_RGB2HSV);
            List<Mat> channels = new ArrayList<>();
            Core.split(hsvImage, channels);

            //Histogram for hue
            Imgproc.calcHist(Arrays.asList(new Mat[]{channels.get(0)}),
                    new MatOfInt(0), new Mat(), histHue, new MatOfInt(BINS),
                    new MatOfFloat(MIN_VALUE, 179));

            //Histogram for saturation
            Imgproc.calcHist(Arrays.asList(new Mat[]{channels.get(1)}),
                    new MatOfInt(0), new Mat(), histSat, new MatOfInt(BINS),
                    new MatOfFloat(MIN_VALUE, MAX_VALUE));

            double sum = Core.sumElems(histHue).val[0];
            double[] values = new double[histHue.height()+histSat.height()];

            RobotLog.dd(TAG, "Sum_Hue %f", sum);
            RobotLog.dd(TAG, "HueMat %dx%d", histHue.width(), histHue.height());

            int k=0;
            for(int i=0; i < histHue.height(); ++i)
            {
                values[k] = histHue.get(i, 0)[0]/sum;
                RobotLog.dd(TAG, "Val_hue %d %f", k, values[k]);
                k++;
            }

            sum = Core.sumElems(histSat).val[0];
            for(int i=0; i < histSat.height(); ++i)
            {
                values[k] = histSat.get(i, 0)[0]/sum;
                k++;
            }

            //0 & 7 Red
            //4 & 5 Blue
            double total = 0.0;

            for(int i=0; i < BINS; i++)
            {
                total+=values[i];
                RobotLog.dd(TAG, "Total %4.2f", total);
            }
            double red = values[0] + values[7];
            double blu = values[4] + values[5];
            double redp = red/total;
            double blup = blu/total;

            if      (redp >= THRESHOLD && redp > blup) foundColor = JewelColor.RED;
            else if (blup >= THRESHOLD && blup > redp) foundColor = JewelColor.BLUE;
            else                                       foundColor = JewelColor.NONE;
        }

        return jewelColor;
    }


    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    @Override public void runOpMode() {

        initOpenCv();
        initVuforia();
        boolean useLight = true;

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        relicTrackables.activate();

        boolean lightOn = false;
        CameraDevice.getInstance().setFlashTorchMode(false);

        if(useLight)
        {
            CameraDevice.getInstance().setFlashTorchMode(true);
            lightOn = true;
        }

        while (opModeIsActive())
        {
            /**
             * See if any of the instances of {@link relicTemplate} are currently visible.
             * {@link RelicRecoveryVuMark} is an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
             * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
             */
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", "%s visible", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
                if (pose != null)
                {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;

                    RobotLog.dd(TAG, "KEY=%s POS=%6.2f %6.2f %6.2f ROT=%6.2f %6.2f &6.2f",
                            vuMark, tX, tY, tZ, rX, rY, rZ);
                }

                if(jewelColor != NONE)
                {
                    jewelColor = getJewelColor();
                    if(lightOn) CameraDevice.getInstance().setFlashTorchMode(false);
                }
            }
            else
            {
                telemetry.addData("VuMark", "not visible");
            }

            telemetry.update();
        }
    }

    String format(OpenGLMatrix transformationMatrix)
    {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    private static native void nBitmapToMat2(Bitmap b, long m_addr, boolean unPremultiplyAlpha);
}
