module learn-opencv-c-02 {
	requires javafx.controls;
	requires javafx.fxml;
	
	opens test_package to javafx.graphics, javafx.fxml;
}
