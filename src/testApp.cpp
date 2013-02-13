#include "testApp.h"

//--------------------------------------------------------------
void testApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);
	
	fingerMovie.loadMovie("MachineBabyTest-Trim2.mov");
	fingerMovie.play();
	fingerMovie.setVolume(0);
    
	// enable depth->rgb image calibration
	kinect.setRegistration(true);
    
	kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // disable video image (faster fps)
	kinect.open();
	
#ifdef USE_TWO_KINECTS
	kinect2.init();
	kinect2.open();
#endif
	
	colorImg.allocate(kinect.width, kinect.height);
	hsb.allocate(kinect.width, kinect.height);
	hue.allocate(kinect.width, kinect.height);
	sat.allocate(kinect.width, kinect.height);
	bri.allocate(kinect.width, kinect.height);
	grayImage.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);
	colorFilteredImage.allocate(kinect.width, kinect.height);
	
	//////////////////////
	redFilteredImage.allocate(kinect.width, kinect.height);
	greenFilteredImage.allocate(kinect.width, kinect.height);
	blueFilteredImage.allocate(kinect.width, kinect.height);
	
	redFilteredImageTwo.allocate(kinect.width, kinect.height);
	greenFilteredImageTwo.allocate(kinect.width, kinect.height);
	blueFilteredImageTwo.allocate(kinect.width, kinect.height);
	
	//////////////////////
	
	subtractionFrame.allocate(kinect.width, kinect.height);
	colorImgDiff.allocate(kinect.width, kinect.height);
	subtractionFrameDiff.allocate(kinect.width, kinect.height);
	briDiff.allocate(kinect.width, kinect.height);
	hueDiff.allocate(kinect.width, kinect.height);
	satDiff.allocate(kinect.width, kinect.height);
	diff.allocate(kinect.width, kinect.height);
	
	subtractionFrame.set(0,0,0);
	nearThreshold = 230;
	farThreshold = 70;
	hueMinR = 165;
	hueMaxR = 255;
	hueMinG = 46;
	hueMaxG = 66;
	hueMinB = 17;
	hueMaxB = 31; //yellow
	
	hueMinRTwo = 141;
	hueMaxRTwo = 150; //purple
	hueMinGTwo = 46;
	hueMaxGTwo = 66;
	hueMinBTwo = 46;
	hueMaxBTwo = 66;
	
	blobMin = 15;
	blobMax = kinect.width / 3;
	
	bThreshWithOpenCV = true;
	
	ofSetFrameRate(60);
	
	// zero the tilt on startup
	angle = 0;
	kinect.setCameraTiltAngle(angle);
	
	// start from the front
	bDrawPointCloud = false;
	
	//wilson@kinoko4eggs.com
	
	// Play an image
	frameByframe = false;
	
	//Setup OSC Communication
	oscSender.setup("127.0.0.1", 57120);
	
}

//--------------------------------------------------------------
void testApp::update() {
	
	//ofBackground(100, 0, 255);
	ofBackground(50, 50, 50);
	
	kinect.update();
	fingerMovie.idleMovie();
	
	// there is a new frame and we are connected
//	if(kinect.isFrameNew()) {
	if(kinect.isFrameNew()) {

		/*
		// Create grayscale image matrix
		//unsigned char greyPixels[kinect.width * kinect.height];
		//for(int i = 0; i < kinect.width*kinect.height; i++){
		//	greyPixels[i] = kinect.getPixels()[3*i+1]; // using the red value, use rgbPixels[3*i+1] for green, etc..
		//}
		
		// load grayscale image from the kinect source
		grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
		
		// we do two thresholds - one for the far plane and one for the near plane
		// we then do a cvAnd to get the pixels which are a union of the two thresholds
		if(bThreshWithOpenCV) {
			grayThreshNear = grayImage;
			grayThreshFar = grayImage;
			grayThreshNear.threshold(nearThreshold, true);
			grayThreshFar.threshold(farThreshold);
			cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
		} else {
			
			// or we do it ourselves - show people how they can work with the pixels
			unsigned char * pix = grayImage.getPixels();
			
			int numPixels = grayImage.getWidth() * grayImage.getHeight();
			for(int i = 0; i < numPixels; i++) {
				if(pix[i] < nearThreshold && pix[i] > farThreshold) {
				pix[i] = 255;
				} else {
					pix[i] = 0;
				}
			}
		}
		 
		 grayImage.flagImageChanged();
		 */
		
		//colorImg.setFromPixels(kinect.getPixels(), kinect.width, kinect.height);
		colorImg.setFromPixels(kinect.getPixels(), kinect.width, kinect.height);

		//Blur and contrast the image
		//colorImg.dilate();
		//colorImg.dilate();
		colorImg.blurGaussian(15);
		//colorImg.blurGaussian(30);
		colorImg.dilate();
		//colorImg.erode();
		
		colorImg.flagImageChanged();
		
		subtractionFrameDiff = subtractionFrame;
		subtractionFrameDiff.convertRgbToHsv();
		subtractionFrameDiff.convertToGrayscalePlanarImages(hueDiff, satDiff,briDiff);

		// update the cv images		
		colorImg.flagImageChanged();
		subtractionFrameDiff.flagImageChanged();

		//duplicate rgb
        hsb = colorImg;
        
        //convert to hsb
        hsb.convertRgbToHsv();
        
        //store the three channels as grayscale images
        hsb.convertToGrayscalePlanarImages(hue, sat, bri);
		hsb.flagImageChanged();
		
		diff.absDiff(hueDiff,hue);
		
		//Setup separate images to filter for each color
		redFilteredImage = colorFilteredImage;
		greenFilteredImage = colorFilteredImage;
		blueFilteredImage = colorFilteredImage;
		

		//BLUE
        //filter image based on the hue value were looking for
        for (int i=0; i<colorFilteredImage.width*colorFilteredImage.height; i++) {
            blueFilteredImage.getPixels()[i] = (ofInRange(diff.getPixels()[i],hueMinB,hueMaxB)) ? 255 : 0;
			//            blueFilteredImage.getPixels()[i] = (ofInRange(diff.getPixels()[i],hueMinB,hueMaxB) && ofInRange(sat.getPixels()[i],125,255)) ? 255 : 0;
        }
		blueFilteredImage.flagImageChanged();
		
		// find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
		// also, find holes is set to true so we will get interior contours as well....
		blueContourFinder.findContours(blueFilteredImage, blobMin * blobMin, blobMax*blobMax, 1, false, 0);
		
		for (int i = 0; i < blueContourFinder.blobs.size(); i++) {

			ofxOscMessage oscMessage;
			oscMessage.setAddress( "/blueBlob");
			oscMessage.addIntArg(i);
			oscMessage.addIntArg(blueContourFinder.blobs[i].centroid.x);
			oscMessage.addIntArg(blueContourFinder.blobs[i].centroid.y);
			
			oscSender.sendMessage(oscMessage);
		}
		
		
		//GREEN
		//filter image based on the hue value were looking for
        for (int i=0; i<colorFilteredImage.width*colorFilteredImage.height; i++) {
            greenFilteredImage.getPixels()[i] = ofInRange(diff.getPixels()[i],hueMinG,hueMaxG) ? 255 : 0;
        }
		greenFilteredImage.flagImageChanged();
		
		greenContourFinder.findContours(greenFilteredImage, blobMin * blobMin, blobMax*blobMax, 1, false, 1);
		
		for (int i = 0; i < greenContourFinder.blobs.size(); i++) {
			int lpCentroidX;

				lpCentroidX = greenContourFinder.blobs[i].centroid.x;

				ofxOscMessage oscMessage;
				oscMessage.setAddress( "/greenBlob");
				oscMessage.addIntArg(i);
				oscMessage.addIntArg(lpCentroidX);
				oscMessage.addIntArg(greenContourFinder.blobs[i].centroid.y);
				
				oscSender.sendMessage(oscMessage);
			
		}
		
		//RED
		//filter image based on the hue value were looking for
        for (int i=0; i<colorFilteredImage.width*colorFilteredImage.height; i++) {
            redFilteredImage.getPixels()[i] = ofInRange(diff.getPixels()[i],hueMinR,hueMaxR) ? 255 : 0;
        }
		redFilteredImage.flagImageChanged();
		
		redContourFinder.findContours(redFilteredImage, blobMin * blobMin, blobMax*blobMax, 1, false, 1);
		
		for (int i = 0; i < redContourFinder.blobs.size(); i++) {
			int lpCentroidX;
			lpCentroidX = redContourFinder.blobs[i].centroid.x;

			ofxOscMessage oscMessage;
			oscMessage.setAddress( "/redBlob");
			oscMessage.addIntArg(i);
			oscMessage.addIntArg(lpCentroidX);
			oscMessage.addIntArg(redContourFinder.blobs[i].centroid.y);
			
			oscSender.sendMessage(oscMessage);
			
		}
		
		//BLUE TWO
        //filter image based on the hue value were looking for
        for (int i=0; i<colorFilteredImage.width*colorFilteredImage.height; i++) {
            blueFilteredImageTwo.getPixels()[i] = (ofInRange(diff.getPixels()[i],hueMinBTwo,hueMaxBTwo) ) ? 255 : 0;
        }
		blueFilteredImageTwo.flagImageChanged();
		
		// find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
		// also, find holes is set to true so we will get interior contours as well....
		blueContourFinderTwo.findContours(blueFilteredImageTwo, blobMin * blobMin, blobMax*blobMax, 1, false, 0);
		
		for (int i = 0; i < blueContourFinderTwo.blobs.size(); i++) {
			
			ofxOscMessage oscMessage;
			oscMessage.setAddress( "/yellowBlob");
			oscMessage.addIntArg(i);
			oscMessage.addIntArg(blueContourFinderTwo.blobs[i].centroid.x);
			oscMessage.addIntArg(blueContourFinderTwo.blobs[i].centroid.y);
			
			oscSender.sendMessage(oscMessage);
		}
		
		
		//GREEN TWO
		//filter image based on the hue value were looking for
        for (int i=0; i<colorFilteredImage.width*colorFilteredImage.height; i++) {
            greenFilteredImageTwo.getPixels()[i] = ofInRange(diff.getPixels()[i],hueMinGTwo,hueMaxGTwo) ? 255 : 0;
        }
		greenFilteredImageTwo.flagImageChanged();
		
		greenContourFinderTwo.findContours(greenFilteredImageTwo, blobMin * blobMin, blobMax*blobMax, 1, false, 1);
		
		for (int i = 0; i < greenContourFinderTwo.blobs.size(); i++) {
			int lpCentroidX;
			
			lpCentroidX = greenContourFinderTwo.blobs[i].centroid.x;
			
			ofxOscMessage oscMessage;
			oscMessage.setAddress( "/orangeBlob");
			oscMessage.addIntArg(i);
			oscMessage.addIntArg(lpCentroidX);
			oscMessage.addIntArg(greenContourFinderTwo.blobs[i].centroid.y);
			
			oscSender.sendMessage(oscMessage);
			
		}
		
		//RED TWO
		//filter image based on the hue value were looking for
        for (int i=0; i<colorFilteredImage.width*colorFilteredImage.height; i++) {
            redFilteredImageTwo.getPixels()[i] = ofInRange(diff.getPixels()[i],hueMinRTwo,hueMaxRTwo) ? 255 : 0;
        }
		redFilteredImageTwo.flagImageChanged();
		
		redContourFinderTwo.findContours(redFilteredImageTwo, blobMin * blobMin, blobMax*blobMax, 1, false, 1);
		
		for (int i = 0; i < redContourFinderTwo.blobs.size(); i++) {
			int lpCentroidX;
			lpCentroidX = redContourFinderTwo.blobs[i].centroid.x;
			
			ofxOscMessage oscMessage;
			oscMessage.setAddress( "/yellowBlob");
			oscMessage.addIntArg(i);
			oscMessage.addIntArg(lpCentroidX);
			oscMessage.addIntArg(redContourFinderTwo.blobs[i].centroid.y);
			
			oscSender.sendMessage(oscMessage);
			
		}
		
	}
	
#ifdef USE_TWO_KINECTS
	kinect2.update();
#endif
}

//--------------------------------------------------------------
void testApp::draw() {
	
	ofSetColor(255, 255, 255);
	
	if(bDrawPointCloud) {
		easyCam.begin();
		drawPointCloud();
		easyCam.end();
	} else {
		// draw from the live kinect
		//kinect.drawDepth(10, 10, 400, 300);
		//fingerMovie.draw(10,10,400,300);
		colorImg.draw(10,70,400,300);
		subtractionFrame.draw(420, 70, 400, 300);
		
		redFilteredImage.draw(10, 390, 200, 150);
		greenFilteredImage.draw(220,390,200,150);
		blueFilteredImage.draw(430,390,200,150);
		
		redContourFinder.draw(10, 390, 200, 150);
		greenContourFinder.draw(220,390,200,150);
		blueContourFinder.draw(430,390,200,150);
		
		redFilteredImageTwo.draw(10, 550, 200, 150);
		greenFilteredImageTwo.draw(220,550,200,150);
		blueFilteredImageTwo.draw(430,550,200,150);
		
		redContourFinderTwo.draw(10, 550, 200, 150);
		greenContourFinderTwo.draw(220,550,200,150);
		blueContourFinderTwo.draw(430,550,200,150);

		
#ifdef USE_TWO_KINECTS
		kinect2.draw(420, 320, 400, 300);
#endif
	}
	
	// draw instructions
	ofSetColor(255, 255, 255);
	stringstream reportStream;
	reportStream //<< "accel is: " << ofToString(kinect.getMksAccel().x, 2) << " / "
	//<< ofToString(kinect.getMksAccel().y, 2) << " / "
	//<< ofToString(kinect.getMksAccel().z, 2) << endl
	//<< "press p to switch between images and point cloud, rotate the point cloud with the mouse" << endl
	//<< "using opencv threshold = " << bThreshWithOpenCV <<" (press spacebar)" << endl
	//<< "set near threshold " << nearThreshold << " (press: + -)" << endl
	//<< "set far threshold " << farThreshold << " (press: < >) num blobs found " << blueContourFinder.nBlobs
	//<< ", fps: " << ofGetFrameRate() << endl
	//<< "press l to close the connection and o to open it again, connection is: " << kinect.isConnected() << endl
	//<< "press UP and DOWN to change the tilt angle: " << angle << " degrees" << endl
	<< "R (q,w,e,r):(" << hueMinR << "," << hueMaxR << ")" << endl
	<< "G (a,s,d,f):(" << hueMinG << "," << hueMaxG << ")" << endl
	<< "B (z,x,c,v):(" << hueMinB << "," << hueMaxB << ")" << endl
	
	<< "R2 (u,i,o,p):(" << hueMinRTwo << "," << hueMaxRTwo << ")" << endl
	<< "G2 (j,k,l,;):(" << hueMinGTwo << "," << hueMaxGTwo << ")" << endl
	<< "B2 (m,<,>,?):(" << hueMinBTwo << "," << hueMaxBTwo << ")" << endl
	<< endl 
	<< "Blob (1,2,3,4):(" << blobMin << "," << blobMax << ")" << endl;
	//<< "Hit space to capture subtraction frame" << endl;
	ofDrawBitmapString(reportStream.str(),640,400);

	ofSetColor(200, 0, 0);
	deathFont.loadFont("DeathDevil.ttf", 52);
	deathFont.setLetterSpacing(0.85);
	deathFont.drawString("DOGCORE", 8, 50);
	ofSetColor(0, 0, 0);
	deathFont.drawString("DOGCORE", 10, 52);
}

void testApp::drawPointCloud() {
	int w = 640;
	int h = 480;
	ofMesh mesh;
	mesh.setMode(OF_PRIMITIVE_POINTS);
	int step = 2;
	for(int y = 0; y < h; y += step) {
		for(int x = 0; x < w; x += step) {
			if(kinect.getDistanceAt(x, y) > 0) {
				mesh.addColor(kinect.getColorAt(x,y));
				mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
			}
		}
	}
	glPointSize(3);
	ofPushMatrix();
	// the projected points are 'upside down' and 'backwards' 
	ofScale(1, -1, -1);
	ofTranslate(0, 0, -1000); // center the points a bit
	glEnable(GL_DEPTH_TEST);
	mesh.drawVertices();
	glDisable(GL_DEPTH_TEST);
	ofPopMatrix();
}

//--------------------------------------------------------------
void testApp::exit() {
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
	
#ifdef USE_TWO_KINECTS
	kinect2.close();
#endif
}

//--------------------------------------------------------------
void testApp::keyPressed (int key) {
	switch (key) {
		case ' ':
			//bThreshWithOpenCV = !bThreshWithOpenCV;
			subtractionFrame.setFromPixels(colorImg.getPixels(), colorImg.width, colorImg.height);
			break;
			
		/*case'l':
			bDrawPointCloud = !bDrawPointCloud;
			break;
			
		case '>':
		case '.':
			farThreshold ++;
			if (farThreshold > 255) farThreshold = 255;
			break;
			
		case '<':
		case ',':
			farThreshold --;
			if (farThreshold < 0) farThreshold = 0;
			break;
			
		case '+':
		case '=':
			nearThreshold ++;
			if (nearThreshold > 255) nearThreshold = 255;
			break;
			
		case '-':
			nearThreshold --;
			if (nearThreshold < 0) nearThreshold = 0;
			break;
			
		//case 'w':
		//	kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
		//	break;
			
		case 'o':
			kinect.setCameraTiltAngle(angle); // go back to prev tilt
			kinect.open();
			break;
			
		case 'p':
			kinect.setCameraTiltAngle(0); // zero the tilt
			kinect.close();
			break;
		*/	
		case OF_KEY_UP:
			angle++;
			if(angle>30) angle=30;
			kinect.setCameraTiltAngle(angle);
			break;
			
		case OF_KEY_DOWN:
			angle--;
			if(angle<-30) angle=-30;
			kinect.setCameraTiltAngle(angle);
			break;
			
		case 'w':
			if (hueMinR < hueMaxR) {
				hueMinR++;
			}
			break;
			
		case 'q':
			if (hueMinR > 0) {
				hueMinR--;
			}
			break;
			
		case 'r':
			if (hueMaxR < 255) {
				hueMaxR++;
			}
			break;
			
		case 'e':
			if (hueMaxR > hueMinR) {
				hueMaxR--;
			}
			break;
		
		case 's':
			if (hueMinG < hueMaxG) {
				hueMinG++;
			}
			break;
			
		case 'a':
			if (hueMinG > 0) {
				hueMinG--;
			}
			break;
			
		case 'f':
			if (hueMaxG < 255) {
				hueMaxG++;
			}
			break;
			
		case 'd':
			if (hueMaxG > hueMinG) {
				hueMaxG--;
			}
			break;
			
		case 'x':
			if (hueMinB < hueMaxB) {
				hueMinB++;
			}
			break;
			
		case 'z':
			if (hueMinB > 0) {
				hueMinB--;
			}
			break;
			
		case 'v':
			if (hueMaxB < 255) {
				hueMaxB++;
			}
			break;
			
		case 'c':
			if (hueMaxB > hueMinB) {
				hueMaxB--;
			}
			break;
			
			
		// Hues Two
		case 'i':
			if (hueMinRTwo < hueMaxRTwo) {
				hueMinRTwo++;
			}
			break;
			
		case 'u':
			if (hueMinRTwo > 0) {
				hueMinRTwo--;
			}
			break;
			
		case 'p':
			if (hueMaxRTwo < 255) {
				hueMaxRTwo++;
			}
			break;
			
		case 'o':
			if (hueMaxRTwo > hueMinRTwo) {
				hueMaxRTwo--;
			}
			break;
			
		case 'k':
			if (hueMinGTwo < hueMaxGTwo) {
				hueMinGTwo++;
			}
			break;
			
		case 'j':
			if (hueMinGTwo > 0) {
				hueMinGTwo--;
			}
			break;
			
		case ';':
			if (hueMaxGTwo < 255) {
				hueMaxGTwo++;
			}
			break;
			
		case 'l':
			if (hueMaxGTwo > hueMinGTwo) {
				hueMaxGTwo--;
			}
			break;
			
		case ',':
			if (hueMinBTwo < hueMaxBTwo) {
				hueMinBTwo++;
			}
			break;
			
		case 'm':
			if (hueMinBTwo > 0) {
				hueMinBTwo--;
			}
			break;
			
		case '/':
			if (hueMaxBTwo < 255) {
				hueMaxBTwo++;
			}
			break;
			
		case '.':
			if (hueMaxBTwo > hueMinBTwo) {
				hueMaxBTwo--;
			}
			break;
			
			////////////////////
			
			
		case '2':
			if (blobMin < blobMax) {
				blobMin++;
			}
			break;
			
		case '1':
			if (blobMin > 0) {
				blobMin--;
			}
			break;
			
		case '4':
			if (blobMax < (kinect.width*kinect.height)) {
				blobMax++;
			}
			break;
			
		case '3':
			if (blobMax > blobMin) {
				blobMax--;
			}
			break;
	}
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button)
{
	
}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h)
{}
