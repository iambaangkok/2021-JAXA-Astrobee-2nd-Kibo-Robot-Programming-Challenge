package jp.jaxa.iss.kibo.rpc.sampleapk;

import gov.nasa.arc.astrobee.Kinematics;
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
import jp.jaxa.iss.kibo.rpc.api.types.PointCloud;
// astrobee library
import android.graphics.Bitmap;
import android.os.SystemClock;
import android.util.Log;
// android library
import com.google.zxing.BinaryBitmap;
import com.google.zxing.LuminanceSource;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.qrcode.QRCodeReader;
// zxing library
import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Rect;
import org.opencv.objdetect.QRCodeDetector;
import static org.opencv.android.Utils.matToBitmap;
import org.opencv.calib3d.Calib3d;
import org.opencv.android.Utils;
// opencv library
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Scanner;
// java library

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {

    final int LOOP_MAX = 5;

    @Override
    protected void runPlan1(){
        // astrobee is undocked and the mission starts
        api.startMission();

        // move to point A (11.21, -9.8, 4.79, 0) quaternion A (0, 0, -0.707, 0.707)
        moveToWrapper(11.21, -9.8, 4.79, 0, 0, -0.707, 0.707, 1, 5);

        // scan QR Code to get point A' (qrData[0], qrData[1], qrData[2]) quaternion A' (0, 0, -0.707, 0.707) KOZ pattern (qrData[3])
        double[] qrData = readQRCode(11.21, -9.8, 4.79, 0, 0, -0.707, 0.707, 1);
        moveToWrapper(qrData[0], qrData[1], qrData[2],0, 0, -0.707, 0.707, 2, 5);

        api.reportMissionCompletion();
    }

    private void moveToWrapper(double px, double py, double pz,
                               double qx, double qy, double qz,
                               double qw, int moveNumber, int loopMax){
        Log.d("moveToWrapper[moveNumber]: ", String.valueOf(moveNumber));

        final Point point = new Point(px, py, pz);
        final Quaternion quaternion = new Quaternion((float)qx, (float)qy,
                                                     (float)qz, (float)qw);

        Result result = api.moveTo(point, quaternion, true);

        int loopCounter = 1;

        while(!result.hasSucceeded() || loopCounter < LOOP_MAX){
            Log.d("moveToWrapper[loopCounter]: ", String.valueOf(loopCounter));
            result = api.moveTo(point, quaternion, true);
            ++loopCounter;
        }
        return;
    }

    private void setFlashOn(boolean status){
        Log.d("setFlashOn[status]: ", String.valueOf(status));
        if(status == true){
            api.flashlightControlFront(0.5f);

            try{
                Thread.sleep(1000);
            }catch (InterruptedException e){
                e.printStackTrace();
            }
        }else{
            api.flashlightControlFront(0f);
        }
        return;
    }

    private void cropImage(Mat sourceImage, Mat targetMat, Rect roi){
        Log.d("cropImage[roi]: ", String.valueOf(roi));
        Mat croppedImage = new Mat(sourceImage,roi);
        croppedImage.copyTo(targetMat);
        return;
    }

    private static void cropImagePercent(Mat sourceImage, Mat targetMat, double percent, double xPercent, double yPercent){
        Log.d("cropImagePercent[status]: " ,"start");

        double width = sourceImage.size().width;
        double height = sourceImage.size().height;
        double newWidth = width*percent/100;
        double newHeight = height*percent/100;
        double newX = width*xPercent/100;
        double newY = height*yPercent/100;

        Rect newROI = new Rect((int)newX,(int)newY,(int)newWidth,(int)newHeight);
        Log.d("cropImagePercent[newROI]: " , newROI.toString());
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
        Log.d("resizeImage[status]: " ,"start");
        Mat resizedImage = new Mat();
        Size sz = new Size(width,height);
        Imgproc.resize( sourceImage, resizedImage, sz );

        return resizedImage;
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
        Log.d("undistortImage[status]: ", "start");
        double[][] navCamIntrinsics = api.getNavCamIntrinsics();
        Log.d("undistortImage[status]: ", "got navCamIntrinsics " + Arrays.deepToString(navCamIntrinsics));

        int row = 0, col = 0;
        Mat cameraMat = new Mat(3, 3, CvType.CV_32FC1);
        cameraMat.put(row, col, navCamIntrinsics[0]);
        Log.d("undistortImage[status]: ", "cameraMat put " + cameraMat.toString() + " " + cameraMat.dump());

        row = 0; col = 0;
        Mat distortionCoeff = new Mat(1, 5, CvType.CV_32FC1);
        distortionCoeff.put(0,0,navCamIntrinsics[1]);
        Log.d("undistortImage[status]: ", "distortCoeff put " + distortionCoeff.toString() + " " + distortionCoeff.dump());

        int test = 1;
        Mat undistortedPic = new Mat(1280, 960, CvType.CV_8UC1);

        Log.d("undistortImage[sourceImage.size()]: ", sourceImage.size().toString());
        Log.d("undistortImage[cameraMat.size()]: ",  cameraMat.size().toString());
        Log.d("undistortImage[distortionCoeff.size()]: ",  distortionCoeff.size().toString());
        Size sourceImageSize = sourceImage.size();
        if(test == 1){
            Mat newCameraMat = Calib3d.getOptimalNewCameraMatrix(cameraMat, distortionCoeff, sourceImageSize, 1, sourceImageSize, roi);
            Log.d("undistortImage[status]: ", "got optimalNewCamMat");
            Log.d("undistortImage[status]: ", "got optimalNewCamMat " + newCameraMat.dump());
            Log.d("undistortImage[newCameraMat.size()]: ", newCameraMat.size().toString());

            Imgproc.undistort(sourceImage,undistortedPic,cameraMat,distortionCoeff,newCameraMat);
            Log.d("undistortImage[status]: ", "finished undistort");
        }else if(test == 2){ //This method crashes the program
            Imgproc.undistort(sourceImage, undistortedPic, cameraMat, distortionCoeff);
            Log.d("undistortImage[status]: ", "finished Imgproc.undistort");
        }

        return undistortedPic;
    }

    private double[] readQRCode(double px, double py, double pz, double qx, double qy, double qz, double qw, int qrnumber){

        String qrData = null;
        int loopCount = 0;
        double result_x = 0, result_y = 0, result_z = 0;
        int kozPattern = 1;
        final int LOOP_MAX = 3;
        while(qrData == null && loopCount < LOOP_MAX){
            Log.d("QR[status]:", " start");
            long start_time = SystemClock.elapsedRealtime();

            // stabilize
            moveToWrapper(px, py, pz, qx, qy, qz, qw, -1, 3);

            // get nav cam pic
            setFlashOn(true);
            Mat pic = api.getMatNavCam();

            // undistort fisheye
            Rect roi = new Rect();
            Mat undistortedPic = undistortImage(pic,roi);

            // crop image to ROI
            Mat croppedPic = new Mat();
            cropImage(undistortedPic, croppedPic, roi);

            Mat croppedPic2 = new Mat();
            cropImagePercent(croppedPic, croppedPic2, 30, 50, 60);

            // enlarge image for easier scanning
            Bitmap bitmapFromMat = Bitmap.createBitmap((int)croppedPic2.size().width, (int)croppedPic2.size().height, Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(croppedPic2,bitmapFromMat);

            Bitmap bitmap = Bitmap.createBitmap(bitmapFromMat, 480, 453, 489, 515);
            Mat resizedImage = new Mat();
            resizedImage = resizeImage(croppedPic2, 1920, 1440);

            // scan QR
            Log.d("QR[status]:", " scanning QR");
            int test = 1;
            if(test == 1){

                int[] pixels = new int[bitmap.getWidth() * bitmap.getHeight()];
                bitmap.getPixels(pixels, 0, bitmap.getWidth(), 0, 0, bitmap.getWidth(), bitmap.getHeight());

                LuminanceSource source = new RGBLuminanceSource(bitmap.getWidth(), bitmap.getHeight(), pixels);
                BinaryBitmap bitmapToRead = new BinaryBitmap(new HybridBinarizer(source));
                Log.d("QR[status]:", " got bitmapToRead");
                try
                {
                    com.google.zxing.Result result = new QRCodeReader().decode(bitmapToRead);
                    qrData = result.getText();
                    Log.d("QR[status]:", " Detected");

                    // Format : "p":<pattern>,"x":<x>,"y":<y>,"z":<z>
                    Scanner s = new Scanner(qrData);
                    kozPattern = s.nextInt();
                    result_x = s.nextDouble();
                    result_y = s.nextDouble();
                    result_z = s.nextDouble();
                }
                catch (Exception e)
                {
                    Log.d("QR[status]:", " Not detected");
                }
            }else if(test == 2){
                QRCodeDetector detector = new QRCodeDetector();
                Log.d("QR[loopCounter]: ", String.valueOf(loopCount));
                qrData = detector.detectAndDecode(pic);
                Log.d("QR[qrData]: ", qrData);
                if(qrData != null){
                    Scanner s = new Scanner(qrData);
                    kozPattern = s.nextInt();
                    result_x = s.nextDouble();
                    result_y = s.nextDouble();
                    result_z = s.nextDouble();
                }
            }


            long stop_time = SystemClock.elapsedRealtime();

            Log.d("QR[loopCount]:", " " + loopCount);
            Log.d("QR[total_time]:"," "+ (stop_time-start_time)/1000);
            loopCount++;
            setFlashOn(false);
        }

        Log.d("QR[qrData]:", qrData);

        api.sendDiscoveredQR(qrData);
        return new double[] {result_x, result_y, result_z, kozPattern};
    }




}

