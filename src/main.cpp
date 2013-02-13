#include "testApp.h"
#include "ofAppGlutWindow.h"

int main() {
	ofAppGlutWindow window;
	ofSetupOpenGL(&window, 825, 718, OF_WINDOW);
	ofSetWindowTitle("INFANTCORE!!!");
	ofRunApp(new testApp());
}
