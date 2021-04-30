package learn_opencv_d;

import org.opencv.core.Core;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Rect;
import org.opencv.calib3d.Calib3d;
import org.opencv.objdetect.QRCodeDetector;
// opencv library
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Scanner;
// java library
import javafx.application.Application;
import javafx.event.EventHandler;
import javafx.stage.Stage;
import javafx.stage.WindowEvent;
import javafx.scene.Scene;
import javafx.scene.layout.BorderPane;
import javafx.fxml.FXMLLoader;

// OpenCV Basics	

public class Main extends Application {
	
	public static void main(String[] args){
		// load the native OpenCV library
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
	
		
		System.out.println("[image]: "+ "loading");
		Mat image = new Mat(); 
		image = Imgcodecs.imread("D:\\GitHub\\JAXA-2nd-Kibo-RPC\\learn\\learn_opencv\\learn_opencv_d\\src\\PointAQR.PNG", Imgcodecs.IMREAD_GRAYSCALE);
		//System.out.println("[image]: "+ image.dump());
		Rect roi = new Rect(1,1,1,1);
		Mat undistortedPic = undistort(image,roi);
		Imgcodecs.imwrite("D:\\GitHub\\JAXA-2nd-Kibo-RPC\\learn\\learn_opencv\\learn_opencv_d\\src\\undistortedImage.png", undistortedPic);
		System.out.println("[roi]: "+ roi.toString());
		
		Mat croppedPic = new Mat();
        cropImage(undistortedPic, croppedPic, roi);
        Imgcodecs.imwrite("D:\\GitHub\\JAXA-2nd-Kibo-RPC\\learn\\learn_opencv\\learn_opencv_d\\src\\croppedImage.png", croppedPic);
        
        Mat croppedPic2 = new Mat();
        cropImagePercent(croppedPic, croppedPic2, 30, 50, 60);
        Imgcodecs.imwrite("D:\\GitHub\\JAXA-2nd-Kibo-RPC\\learn\\learn_opencv\\learn_opencv_d\\src\\croppedImage2.png", croppedPic2);
        
        Mat resizedImage = new Mat();
        Size sz = new Size(1920,1440);
        Imgproc.resize( croppedPic2, resizedImage, sz );
        Imgcodecs.imwrite("D:\\GitHub\\JAXA-2nd-Kibo-RPC\\learn\\learn_opencv\\learn_opencv_d\\src\\resizedImage.png", resizedImage);
        
        String qrData = readQR(croppedPic);
        
        
		//launch(args);
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

	
	private static void cropImage(Mat sourceImage, Mat targetMat, Rect roi){
        System.out.println("cropImage[roi]: "+ String.valueOf(roi));
        Mat croppedImage = new Mat(sourceImage,roi);
        croppedImage.copyTo(targetMat);
        return;
    }
	
	private static void cropImagePercent(Mat sourceImage, Mat targetMat, double percent, double xPercent, double yPercent){
        System.out.println("cropImagePercent[status]: " + "start");
        
        double width = sourceImage.size().width;
        double height = sourceImage.size().height;
        double newWidth = width*percent/100;
        double newHeight = height*percent/100;
        double newX = width*xPercent/100;
        double newY = height*yPercent/100;
        
        Rect newROI = new Rect((int)newX,(int)newY,(int)newWidth,(int)newHeight);
        
        Mat croppedImage = new Mat(sourceImage,newROI);
        croppedImage.copyTo(targetMat);
        return;
    }
	
	
	public static Mat undistort(Mat sourceImage, Rect roi) {
		System.out.println("undistortImage[status]: "+ "start");
        
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		double[][] navCamIntrinsics  = {{567.229305, 0.0, 659.077221, 0.0, 574.192915, 517.007571, 0.0, 0.0, 1.0}, {-0.216247, 0.03875, -0.010157, 0.001969, 0.0}};
		
		
        System.out.println("undistortImage[status]: "+ "got navCamIntrinsics " + Arrays.deepToString(navCamIntrinsics));
        int row = 0, col = 0;
        Mat cameraMat = new Mat(3, 3, CvType.CV_32FC1);
        cameraMat.put(row, col, navCamIntrinsics[0]);
        System.out.println("undistortImage[status]: "+ "cameraMat put " + cameraMat.toString() + " " + cameraMat.dump());

        row = 0; col = 0;
        Mat distortionCoeff = new Mat(1, 5, CvType.CV_32FC1);
        distortionCoeff.put(row,col,navCamIntrinsics[1]);
        System.out.println("undistortImage[status]: "+ "distortCoeff put " + distortionCoeff.toString() + " " + distortionCoeff.dump());

        int test = 1;
        Mat undistortedPic = new Mat(1280, 960, CvType.CV_8UC1);

        System.out.println("undistortImage[sourceImage.size()]: " +  sourceImage.size().toString());
        System.out.println("undistortImage[cameraMat.size()]: " +  cameraMat.size().toString());
        System.out.println("undistortImage[distortionCoeff.size()]: " +  distortionCoeff.size().toString());
        Size sourceImageSize = sourceImage.size();
        if(test == 1){
            Mat newCameraMat = Calib3d.getOptimalNewCameraMatrix(cameraMat, distortionCoeff, sourceImageSize, 1, sourceImageSize, roi);
            System.out.println("undistortImage[status]: "+ "got optimalNewCamMat");
            System.out.println("undistortImage[status]: "+ "got optimalNewCamMat " + newCameraMat.dump());
            System.out.println("undistortImage[newCameraMat.size()]: " +  newCameraMat.size().toString());

            Calib3d.undistort(sourceImage, undistortedPic, cameraMat, distortionCoeff, newCameraMat);// (sourceImage,undistortedPic,cameraMat,distortionCoeff,newCameraMat);
            System.out.println("undistortImage[status]: "+ "finished fisheye_undistort");
        }else if(test == 2){ //This method crashes the program
            //Imgproc.undistort(sourceImage, undistortedPic, cameraMat, distortionCoeff);
            System.out.println("undistortImage[status]: "+ "finished Imgproc.undistort");
        }
        
        return undistortedPic;
	}
	
	
	
	//@Override
	public void start(Stage primaryStage) {
		try{
			// load the FXML resource
			FXMLLoader loader = new FXMLLoader(getClass().getResource("FirstFX.fxml"));
			// store the root element so that the controllers can use it
			BorderPane rootElement = (BorderPane) loader.load();
			// create and style a scene
			Scene scene = new Scene(rootElement, 800, 600);
			scene.getStylesheets().add(getClass().getResource("application.css").toExternalForm());
			// create the stage with the given title and the previously created
			// scene
			primaryStage.setTitle("JavaFX meets OpenCV");
			primaryStage.setScene(scene);
			// show the GUI
			primaryStage.show();
			
			// set the proper behavior on closing the application
			FXController controller = loader.getController();
			primaryStage.setOnCloseRequest((new EventHandler<WindowEvent>() {
				public void handle(WindowEvent we){
					controller.setClosed();
				}
			}));
		}
		catch (Exception e){
			e.printStackTrace();
		}
	}
	
	
	
}
