package jp.jaxa.iss.kibo.rpc.sampleapk;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
// astrobee library
import android.os.SystemClock;
import android.util.Log;
import android.graphics.Bitmap;
// android library
import com.google.zxing.BinaryBitmap;
import com.google.zxing.LuminanceSource;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.qrcode.QRCodeReader;
import com.google.zxing.DecodeHintType;
// zxing library
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Rect;
import org.opencv.objdetect.QRCodeDetector;
import static org.opencv.android.Utils.matToBitmap;
import org.opencv.calib3d.Calib3d;
// opencv library
import java.util.Arrays;
import java.util.Scanner;
import java.util.HashMap;
import java.util.Map;
// java library

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {

    final int LOOP_MAX = 5;
    static long startTime = 0;
    @Override
    protected void runPlan1(){
        // astrobee is undocked and the mission starts
        api.startMission();
        startTime = getTime();
        // move to point A (11.21, -9.8, 4.79, 0) quaternion A (0, 0, -0.707, 0.707)
        moveToWrapper(11.21, -9.8, 4.79, 0, 0, -0.707, 0.707, 1, 3);

        // scan QR Code to get point A' (qrData[0], qrData[1], qrData[2]) quaternion A' (0, 0, -0.707, 0.707) KOZ pattern (qrData[3])
        double[] qrData = readQRCode(11.21, -9.8, 4.79, 0, 0, -0.707, 0.707, 1);
        moveToWrapper(qrData[0], qrData[1], qrData[2],0, 0, -0.707, 0.707, 2, 3);

        api.reportMissionCompletion();
    }

    private void moveToWrapper(double px, double py, double pz,
                               double qx, double qy, double qz,
                               double qw, int moveNumber, int loopMax){
        Log.d("moveToWrapper[moveNumber]: ", String.valueOf(moveNumber) + getElapsedTimeS());

        final Point point = new Point(px, py, pz);
        final Quaternion quaternion = new Quaternion((float)qx, (float)qy,
                                                     (float)qz, (float)qw);

        Result result = api.moveTo(point, quaternion, true);

        int loopCounter = 1;

        while(!result.hasSucceeded() || loopCounter < LOOP_MAX){
            Log.d("moveToWrapper[loopCounter]: ", String.valueOf(loopCounter) + getElapsedTimeS());
            result = api.moveTo(point, quaternion, true);
            ++loopCounter;
        }
        return;
    }

    private static long getTime(){
        return SystemClock.elapsedRealtime();
    }
    private static long getElapsedTime(){
        return getTime()-startTime;
    }
    private static String getElapsedTimeS(){
        return " _Time_: " + String.valueOf(getElapsedTime());
    }

    private void setFlashOn(boolean status){
        Log.d("setFlashOn[status]: ", String.valueOf(status) + getElapsedTimeS());
        if(status == true){
            api.flashlightControlFront(0.5f);
            Log.d("setFlashOn[status]: ", "brightness = " + String.valueOf(0.5f) + getElapsedTimeS());
            try{
                Log.d("setFlashOn[status]: ", "sleeping");
                Thread.sleep(1000);
            }catch (InterruptedException e){
                Log.d("setFlashOn[status]: ", "interrupted" + getElapsedTimeS());
                e.printStackTrace();
            }
        }else{
            api.flashlightControlFront(0f);
            Log.d("setFlashOn[status]: ", "off" + getElapsedTimeS());
        }
        Log.d("setFlashOn[status]: ", "done" + getElapsedTimeS());
        return;
    }

    private void cropImage(Mat sourceImage, Mat targetMat, Rect roi){
        Log.d("cropImage[roi]: ", String.valueOf(roi) + getElapsedTimeS());
        Mat croppedImage = new Mat(sourceImage,roi);
        croppedImage.copyTo(targetMat);
        return;
    }

    private static void cropImagePercent(Mat sourceImage, Mat targetMat, double percent, double xPercent, double yPercent){
        Log.d("cropImagePercent[status]: " ,"start" + getElapsedTimeS());

        double width = sourceImage.size().width;
        double height = sourceImage.size().height;
        double newWidth = width*percent/100;
        double newHeight = height*percent/100;
        double newX = width*xPercent/100;
        double newY = height*yPercent/100;

        Rect newROI = new Rect((int)newX,(int)newY,(int)newWidth,(int)newHeight);
        Log.d("cropImagePercent[newROI]: " , newROI.toString() + getElapsedTimeS() + getElapsedTimeS());
        Mat croppedImage = new Mat(sourceImage,newROI);
        croppedImage.copyTo(targetMat);
        return;
    }

    /*public Bitmap resizeImage(Mat sourceImage, int width, int height) {
        Log.d("resizeImage[status]: ", "start");
        Log.d("resizeImage[widthheight]: ", String.valueOf(width)+" "+ String.valueOf(height));
        Size size = new Size(width, height);
        Imgproc.resize(sourceImage, sourceImage, size);
        Log.d("resizeImage[status]: ", "image resized");

        Bitmap bitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
        matToBitmap(sourceImage, bitmap, false);
        Log.d("resizeImage[status]: ", "bitmap done");
        return bitmap;
    }*/

    public Mat resizeImage(Mat sourceImage, int width, int height){
        Log.d("resizeImage[status]: " ,"start" + getElapsedTimeS());
        Mat resizedImage = new Mat();
        Size sz = new Size(width,height);
        Imgproc.resize( sourceImage, resizedImage, sz );

        return resizedImage;
    }
    public Bitmap resizeImageBitmap(Mat src, int width, int height)
    {
        Size size = new Size(width, height);
        Imgproc.resize(src, src, size);

        Bitmap bitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
        matToBitmap(src, bitmap, false);
        return bitmap;
    }
    public static String readQR(Mat pic){
        System.out.println("readQR[status]: " + " start");
        String content = "";
        QRCodeDetector detector = new QRCodeDetector();
        int loopCounter = 0;
        final int LOOP_MAX = 5;
        while( content.isEmpty() && loopCounter < LOOP_MAX){
            System.out.println("readQR[loopCounter]: " + loopCounter);
            content = detector.detectAndDecode(pic);
            loopCounter++;
        }
        System.out.println("readQR[content]: " + content);
        return content;
    }

    private Mat undistortImage(Mat sourceImage, Rect roi){
        Log.d("undistortImage[status]: ", "start" + getElapsedTimeS());
        double[][] navCamIntrinsics = api.getNavCamIntrinsics();
        Log.d("undistortImage[status]: ", "got navCamIntrinsics " + Arrays.deepToString(navCamIntrinsics) + getElapsedTimeS());

        int row = 0, col = 0;
        Mat cameraMat = new Mat(3, 3, CvType.CV_32FC1);
        cameraMat.put(row, col, navCamIntrinsics[0]);
        Log.d("undistortImage[status]: ", "cameraMat put " + cameraMat.toString() + " " + cameraMat.dump() + getElapsedTimeS());

        row = 0; col = 0;
        Mat distortionCoeff = new Mat(1, 5, CvType.CV_32FC1);
        distortionCoeff.put(row,col,navCamIntrinsics[1]);
        Log.d("undistortImage[status]: ", "distortCoeff put " + distortionCoeff.toString() + " " + distortionCoeff.dump() + getElapsedTimeS());

        int test = 1;
        Mat undistortedPic = new Mat(1280, 960, CvType.CV_8UC1);

        Log.d("undistortImage[sourceImage.size()]: ", sourceImage.size().toString() + getElapsedTimeS());
        Log.d("undistortImage[cameraMat.size()]: ",  cameraMat.size().toString() + getElapsedTimeS());
        Log.d("undistortImage[distortionCoeff.size()]: ",  distortionCoeff.size().toString() + getElapsedTimeS());
        Size sourceImageSize = sourceImage.size();
        if(test == 1){
            Mat newCameraMat = Calib3d.getOptimalNewCameraMatrix(cameraMat, distortionCoeff, sourceImageSize, 1, sourceImageSize, roi);
            Log.d("undistortImage[status]: ", "got optimalNewCamMat" + getElapsedTimeS());
            Log.d("undistortImage[status]: ", "got optimalNewCamMat " + newCameraMat.dump() + getElapsedTimeS());
            Log.d("undistortImage[newCameraMat.size()]: ", newCameraMat.size().toString() + getElapsedTimeS());

            Imgproc.undistort(sourceImage,undistortedPic,cameraMat,distortionCoeff,newCameraMat);
            Log.d("undistortImage[status]: ", "finished undistort" + getElapsedTimeS());
        }else if(test == 2){
            Imgproc.undistort(sourceImage, undistortedPic, cameraMat, distortionCoeff);
            Log.d("undistortImage[status]: ", "finished Imgproc.undistort" + getElapsedTimeS());
        }

        return undistortedPic;
    }

    private double[] readQRCode(double px, double py, double pz, double qx, double qy, double qz, double qw, int qrnumber){

        String qrData = null;
        int loopCount = 0;
        double result_x = -1, result_y = -1, result_z = -1;
        int kozPattern = -1;
        final int LOOP_MAX = 3;
        while(qrData == null && loopCount < LOOP_MAX){
            Log.d("QR[status]:", " start" + getElapsedTimeS());
            long start_time = SystemClock.elapsedRealtime();

            // stabilize
            Log.d("QR[status]:", " stabilizing" + getElapsedTimeS());
            moveToWrapper(px, py, pz, qx, qy, qz, qw, -1, 1);

            // get nav cam pic
            Log.d("QR[status]:", " getting nav cam image" + getElapsedTimeS());
            setFlashOn(true);
            Mat pic = api.getMatNavCam();

            // undistort
            Log.d("QR[status]:", " undistorting" + getElapsedTimeS());
            Rect roi = new Rect();
            Mat undistortedPic = undistortImage(pic,roi);

            // crop image to ROI
            Log.d("QR[status]:", " 1st cropping" + getElapsedTimeS());
            cropImage(undistortedPic, undistortedPic, roi);

            Log.d("QR[status]:", " 2nd cropping" + getElapsedTimeS());
            cropImagePercent(undistortedPic, undistortedPic, 30, 50, 60);

            Log.d("QR[status]:", " enlarging" + getElapsedTimeS());
            Size sz = new Size(1920,1440);
            Imgproc.resize( undistortedPic, undistortedPic, sz );

            Log.d("QR[status]:", " adjusting contrast/brightness" + getElapsedTimeS());
            double contrast = 1;
            int brightness = -50;
            undistortedPic.convertTo(undistortedPic, -1, contrast, brightness);

            Log.d("QR[status]:", " sharpening" + getElapsedTimeS());
            Mat blurredImage = new Mat();
            int kernelSize = 19;
            Imgproc.GaussianBlur(undistortedPic, blurredImage, new Size(kernelSize, kernelSize), 30);
            Core.addWeighted(undistortedPic, 1.5, blurredImage, -0.5, 0, undistortedPic);
            Core.addWeighted(undistortedPic, 1.5, blurredImage, -0.5, 0, undistortedPic);
            Core.addWeighted(undistortedPic, 1.5, blurredImage, -0.5, 0, undistortedPic);
            Core.addWeighted(undistortedPic, 1.5, blurredImage, -0.5, 0, undistortedPic);

            final Size sizeA = undistortedPic.size();

            Log.d("QR[status]:", " 3rd cropping" + getElapsedTimeS());
            Rect roi2 = new Rect((int)(sizeA.width*0.08),(int)(sizeA.height*0.1),(int)(sizeA.width*0.6),(int)(sizeA.height*0.8));
            cropImage(undistortedPic, undistortedPic, roi2);

            Log.d("QR[status]:", " 4th cropping" + getElapsedTimeS());
            cropImagePercent(undistortedPic, undistortedPic, 50, 10, 20);

            blurredImage = new Mat();
            Imgproc.GaussianBlur(undistortedPic, blurredImage, new Size(kernelSize, kernelSize), 30);
            Core.addWeighted(undistortedPic, 1.5, blurredImage, -0.5, 0, undistortedPic);
            blurredImage = new Mat();
            Imgproc.GaussianBlur(undistortedPic, blurredImage, new Size(kernelSize, kernelSize), 30);
            Core.addWeighted(undistortedPic, 1.5, blurredImage, -0.5, 0, undistortedPic);
            blurredImage = new Mat();
            Imgproc.GaussianBlur(undistortedPic, blurredImage, new Size(kernelSize, kernelSize), 30);
            Core.addWeighted(undistortedPic, 1.5, blurredImage, -0.5, 0, undistortedPic);

            /*Log.d("QR[status]:", " resizeImageBitmap" + getElapsedTimeS());
            Bitmap bMap = resizeImageBitmap(undistortedPic, 2000, 1500);
            Log.d("QR[status]:", " intArray" + getElapsedTimeS());
            int[] intArray = new int[bMap.getWidth() * bMap.getHeight()];
            Log.d("QR[status]:", " getPixels" + getElapsedTimeS());
            bMap.getPixels(intArray, 0, bMap.getWidth(), 0, 0, bMap.getWidth(), bMap.getHeight());*/


            // prepare intArray for LuminanceSource
            Log.d("QR[status]:", " getting intArray" + getElapsedTimeS());
            final Size sizeB = undistortedPic.size();
            Log.d("QR[sizeB]:", sizeB.toString() + getElapsedTimeS());
            final int size = (int)(sizeB.width*sizeB.height);
            Log.d("QR[size]:", ""+size + getElapsedTimeS());
            MatOfInt grayscale = new MatOfInt(CvType.CV_32SC1);
            Log.d("QR[status]:", " got matofint" + getElapsedTimeS());
            undistortedPic.convertTo(grayscale,CvType.CV_32SC1);
            Log.d("QR[status]:", " converted" + getElapsedTimeS());
            int[] intArray = new int[(int)(grayscale.total()*grayscale.channels())];
            Log.d("QR[status]:", " getting intArray" + getElapsedTimeS());
            grayscale.get(0,0,intArray);
            Log.d("QR[status]:", " starting loop" + getElapsedTimeS());
            /*for (int i = 0; i < sizeB.height; i++) {
                for (int j = 0; j < sizeB.width; j++) {
                    double[] data = undistortedPic.get(i, j);
                    intArray[(int)(sizeB.width*i + j)] = (int)data[0];
                }
            }*/

            Map<DecodeHintType, String> hints = new HashMap<>();
            hints.put(DecodeHintType.TRY_HARDER, "utf-8");
            // scan QR
            int test = 1;
            //if(test == 1){
                Log.d("QR[status]:", " getting luminanceSource" + getElapsedTimeS());
                LuminanceSource source = new RGBLuminanceSource((int)undistortedPic.size().width, (int)undistortedPic.size().height, intArray);
                Log.d("QR[status]:", " got luminanceSource" + getElapsedTimeS());
                BinaryBitmap bitmap = new BinaryBitmap(new HybridBinarizer(source));
                Log.d("QR[status]:", " got bitmapToRead" + getElapsedTimeS());

                Log.d("QR[status]:", " scanning QR zxing" + getElapsedTimeS());
                try
                {
                    Log.d("QR[status]:", " scanning QR zxing try" + getElapsedTimeS());
                    com.google.zxing.Result result = new QRCodeReader().decode(bitmap, hints);
                    Log.d("QR[status]:", " Detected" + getElapsedTimeS());
                    qrData = result.getText();
                    Log.d("QR[status]:", " got text" + getElapsedTimeS());

                    qrData = qrData.replaceAll("[^0-9,.]", "");
                    // Format : "p":<pattern>,"x":<x>,"y":<y>,"z":<z>
                    Scanner s = new Scanner(qrData);
                    Log.d("QR[status]:", " zxing getting koz" + getElapsedTimeS());
                    String[] multi_contents = qrData.split(",");
                    //System.out.println(multi_contents[0].replaceAll("[^0-9]", ""));
                    kozPattern = Integer.parseInt(multi_contents[0]);
                    Log.d("QR[status]:", " zxing getting x" + getElapsedTimeS());
                    result_x = Double.parseDouble(multi_contents[1]);
                    Log.d("QR[status]:", " zxing getting y" + getElapsedTimeS());
                    result_y = Double.parseDouble(multi_contents[2]);
                    Log.d("QR[status]:", " zxing getting z" + getElapsedTimeS());
                    result_z = Double.parseDouble(multi_contents[3]);
                }
                catch (Exception e)
                {
                    Log.d("QR[status]:", " Not detected" + getElapsedTimeS());
                }

                if(qrData != null){
                    Log.d("QR[qrData]:",
                            String.valueOf(kozPattern) + " " + String.valueOf(result_x) + " "
                                    + String.valueOf(result_y) + " " + String.valueOf(result_z) + getElapsedTimeS());
                }
            //}else if(test == 2){
                /*Log.d("QR[status]:", " scanning QR opencv");
                QRCodeDetector detector = new QRCodeDetector();
                Log.d("QR[loopCounter]: ", String.valueOf(loopCount));
                String result = detector.detectAndDecode(undistortedPic);
                Log.d("QR[qrData]: ", qrData);
                if(result != null){
                    Log.d("QR[status]:", " Detected");
                    qrData = result;
                    qrData = qrData.replaceAll("[^0-9,.]", "");
                    String[] multi_contents = qrData.split(",");
                    System.out.println(multi_contents[0].replaceAll("[^0-9]", ""));
                    kozPattern = Integer.parseInt(multi_contents[0]);
                    Log.d("QR[status]:", " opencv getting x");
                    result_x = Double.parseDouble(multi_contents[1]);
                    Log.d("QR[status]:", " opencv getting y");
                    result_y = Double.parseDouble(multi_contents[2]);
                    Log.d("QR[status]:", " opencv getting z");
                    result_z = Double.parseDouble(multi_contents[3]);
                }*/
            //}


            long stop_time = SystemClock.elapsedRealtime();

            Log.d("QR[loopCount]:", " " + loopCount + getElapsedTimeS());
            Log.d("QR[total_time]:"," "+ (stop_time-start_time)/1000);
            loopCount++;
            setFlashOn(false);
        }

        Log.d("QR[qrData]:", qrData + getElapsedTimeS());

        api.sendDiscoveredQR(qrData);
        return new double[] {result_x, result_y, result_z, kozPattern};
    }




}

