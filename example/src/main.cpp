#include "ofMain.h"
#include "ofApp.h"
#include "ofAppGLFWWindow.h"


int main()
{
    int numScreens = 1;
    int screenWidth = 1920;
    int screenHeight = 1200;

    ofGLFWWindowSettings settings;

    settings.multiMonitorFullScreen = true;
    settings.resizable = false;
    settings.setSize(screenWidth * numScreens, screenHeight);
    settings.windowMode = OF_FULLSCREEN;

    auto window = ofCreateWindow(settings);
    auto app = std::make_shared<ofApp>();

    ofRunApp(window, app);
    return ofRunMainLoop();
}
