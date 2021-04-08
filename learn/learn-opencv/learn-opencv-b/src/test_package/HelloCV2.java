package test_package;

import java.lang.reflect.Field;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

public class HelloCV2 {
    
    public static void main(String[] args){
        try {
            HelloCV2.loadOpenCV_Lib();
            System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
            Mat mat = Mat.eye(3, 3, CvType.CV_8UC1);
            System.out.println("mat = " + mat.dump());
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

        

    public static void loadOpenCV_Lib() throws Exception {
        // get the model
        String model = System.getProperty("sun.arch.data.model");
        // the path the .dll lib location
        String libraryPath = "d:/Code/OpenCV/opencv-4.5.2/build/java/x86/";
        // check for if system is 64 or 32
        if(model.equals("64")) {
            libraryPath = "d:/Code/OpenCV/opencv-4.5.2/build/java/x64/";
        }
        // set the path
        System.setProperty("java.library.path", libraryPath);
        Field sysPath = ClassLoader.class.getDeclaredField("sys_paths");
        sysPath.setAccessible(true);
        sysPath.set(null, null);
        // load the lib
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }
}
