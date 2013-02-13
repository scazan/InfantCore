#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxOSC.h"
#include "ofTrueTypeFont.h"
#include <stack>;


// uncomment this to read from two kinects simultaneously
//#define USE_TWO_KINECTS

class testApp : public ofBaseApp {
public:
	
	void setup();
	void update();
	void draw();
	void exit();
	
	void drawPointCloud();
	
	void keyPressed (int key);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
	
	ofVideoPlayer 		fingerMovie;
	bool                frameByframe;
	ofxKinect kinect;
	ofxOscSender oscSender;
	
#ifdef USE_TWO_KINECTS
	ofxKinect kinect2;
#endif
	
	ofxCvColorImage colorImg, hsb, subtractionFrame, subtractionFrameDiff, colorImgDiff;
	
	ofxCvGrayscaleImage grayImage; // grayscale depth image
	ofxCvGrayscaleImage colorFilteredImage, hue,sat,bri, hueDiff,satDiff,briDiff, diff, 
	redFilteredImage, greenFilteredImage, blueFilteredImage,
	redFilteredImageTwo, greenFilteredImageTwo, blueFilteredImageTwo;
	ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
	ofxCvGrayscaleImage grayThreshFar; // the far thresholded image
	
	ofxCvContourFinder redContourFinder, greenContourFinder, blueContourFinder;
	ofxCvContourFinder redContourFinderTwo, greenContourFinderTwo, blueContourFinderTwo;
	
	bool bThreshWithOpenCV;
	bool bDrawPointCloud;
	
	int nearThreshold;
	int farThreshold;
	
	int angle;
	
	int findHue;
	int hueMinR, hueMaxR;
	int hueMinG, hueMaxG;
	int hueMinB, hueMaxB;
	
	int hueMinRTwo, hueMaxRTwo;
	int hueMinGTwo, hueMaxGTwo;
	int hueMinBTwo, hueMaxBTwo;
	

	int blobMin, blobMax;
	stack<int> lastCentroidX, lastCentroidY;
	ofTrueTypeFont deathFont;
	
    // used for viewing the point cloud
	ofEasyCam easyCam;
	
};
