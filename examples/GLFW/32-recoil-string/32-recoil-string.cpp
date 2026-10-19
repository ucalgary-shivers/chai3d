//==============================================================================
/*
    DEMO:    32-recoil-string.cpp

    C++ port of the miPhysics RecoilString example. A chain of 70 masses
    linked by spring-dampers is anchored to the ground at one end and to a
    heavy mass at the other. The haptic device drives a spherical avatar
    that collides with the masses. The physical model runs at audio rate
    (miniaudio), and the position of one mass is rendered as sound.
*/
//==============================================================================

//------------------------------------------------------------------------------
#define MA_NO_DECODING
#define MA_NO_ENCODING
#define MA_NO_GENERATION
#define MINIAUDIO_IMPLEMENTATION
#include "../../../externals/miniaudio/miniaudio.h"
//------------------------------------------------------------------------------
#include "chai3d.h"
#include "miphysics.h"
//------------------------------------------------------------------------------
#include <GLFW/glfw3.h>
//------------------------------------------------------------------------------
#include <atomic>
#include <string>
//------------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// GENERAL SETTINGS
//------------------------------------------------------------------------------

// fullscreen mode
bool fullscreen = false;

// mirrored display
bool mirroredDisplay = false;

// audio sample rate [Hz] (also the rate of the physical model)
const int audioSampleRate = 44100;

// audio buffer size [frames]; kept small to limit how long the model lock is held
const int audioPeriodFrames = 128;

// target rate of the haptic loop [Hz]
const int hapticRate = 1000;

// model parameters (algorithm units)
const int    nbMass   = 70;
const double massM    = 1.0;
const double massEnd  = 10000000.0;
const double springK  = 0.1;
const double springZ  = 0.001;
const double springL0 = 0.05;
const double massDist = 0.1;
const double fric     = 0.00005;
const double grav     = 0.0;
const double contactDist = 0.9;
const double contactK    = 0.008;
const double contactZ    = 0.001;
const int    ewmaSmooth  = 30;

// name of the mass whose position is rendered as sound
const string listeningPoint = "str4";

// device workspace [m] to model space scaling, and force scaling
const double alpha = 200.0;
const double beta  = 0.3;

// model position of the avatar when the device is at its origin
const double avatarHomeX = 0.5 * massDist * (nbMass + 1);
const double avatarHomeY = -5.0;

// graphic scale [m per model unit] and model point looked at by the camera
const double gfxScale = 0.05;
const double camTargetY = -2.0;


//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in a window display
cCamera* camera;

// a viewport to display the scene viewed by the camera
cViewport* viewport = nullptr;

// a light source to illuminate the objects in the virtual scene
cDirectionalLight* light;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
cGenericHapticDevicePtr hapticDevice;

// maximum force the device can render [N]
double maxDeviceForce = 0.0;

// a colored background
cBackground* background;

// a font for rendering text
cFontPtr font;

// labels displaying the haptic/graphic rates and simulation data
cLabel* labelRates;
cLabel* labelMissed;
cLabel* labelPos;
cLabel* labelForce;
cLabel* labelAudio;

// the physical model
miphysics::PhysicalModel* model = nullptr;

// mass listened to by the audio callback
const miphysics::Mat* listenedMat = nullptr;

// graphic representation of the model
vector<cShapeSphere*> matShapes;
vector<cShapeLine*> linkShapes;

// audio output
ma_device audioDev;
bool audioDevInitialized = false;

// audio state (audio thread only, except currAudio)
double prevSample = 0.0;
double audioOut = 0.0;
atomic<float> currAudio(0.0f);

// data shared between the haptic loop and the graphics loop
atomic<double> posEEx(0.0), posEEy(0.0);
atomic<double> fEEx(0.0), fEEy(0.0);
atomic<int> stepsMissed(0);

// avatar offset in model space; modified by the haptic thread only
double offsetX = avatarHomeX;
double offsetY = avatarHomeY;

// request to re-center the avatar on the current device position
atomic<bool> recenterRequested(false);

// a flag that indicates if the haptic simulation is currently running
bool simulationRunning = false;

// a flag that indicates if the haptic simulation has terminated
bool simulationFinished = true;

// a frequency counter to measure the simulation graphic rate
cFrequencyCounter freqCounterGraphics;

// a frequency counter to measure the simulation haptic rate
cFrequencyCounter freqCounterHaptics;

// haptic thread
cThread* hapticsThread;

// a handle to window display context
GLFWwindow* window = nullptr;

// current size of GLFW window
int windowW = 0;
int windowH = 0;

// current size of GLFW framebuffer
int framebufferW = 0;
int framebufferH = 0;

// swap interval for the display context (vertical synchronization)
int swapInterval = 1;


//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// callback when the window is resized
void onWindowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);

// callback when the window framebuffer is resized
void onFrameBufferSizeCallback(GLFWwindow* a_window, int a_width, int a_height);

// callback when an error GLFW occurs
void onErrorCallback(int a_error, const char* a_description);

// callback when a key is pressed
void onKeyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);

// callback when window content scaling is modified
void onWindowContentScaleCallback(GLFWwindow* a_window, float a_xscale, float a_yscale);

// audio callback: steps the physical model once per output frame
void onAudioData(ma_device* a_device, void* a_output, const void* a_input, ma_uint32 a_frames);

// builds the RecoilString physical model
void createModel();

// builds the graphic objects matching the model
void createModelShapes();

// moves the graphic objects to the current model state
void updateModelShapes();

// this function renders the scene
void renderGraphics(void);

// this function contains the main haptics simulation loop
void renderHaptics(void);

// this function closes the application
void close(void);


//==============================================================================

int main(int argc, char* argv[])
{
    //--------------------------------------------------------------------------
    // INITIALIZATION
    //--------------------------------------------------------------------------

    cout << endl;
    cout << "-----------------------------------" << endl;
    cout << "CHAI3D" << endl;
    cout << "Demo: 32-recoil-string (miPhysics)" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[o] - Re-center the avatar on the current device position" << endl;
    cout << "[f] - Enable/Disable full screen mode" << endl;
    cout << "[m] - Enable/Disable vertical mirroring" << endl;
    cout << "[q] - Exit application" << endl;
    cout << endl << endl;


    //--------------------------------------------------------------------------
    // OPEN GL - WINDOW DISPLAY
    //--------------------------------------------------------------------------

    // initialize GLFW library
    if (!glfwInit())
    {
        cout << "failed initialization" << endl;
        cSleepMs(1000);
        return 1;
    }

    // set GLFW error callback
    glfwSetErrorCallback(onErrorCallback);

    // compute desired size of window
    const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
    windowW = 0.8 * mode->height;
    windowH = 0.5 * mode->height;
    int x = 0.5 * (mode->width - windowW);
    int y = 0.5 * (mode->height - windowH);

    // set OpenGL version
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

    // enable double buffering
    glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_TRUE);

    // set the desired number of samples to use for multisampling
    glfwWindowHint(GLFW_SAMPLES, 4);

    // specify that window should be resized based on monitor content scale
    glfwWindowHint(GLFW_SCALE_TO_MONITOR, GLFW_TRUE);

    glfwWindowHint(GLFW_STEREO, GL_FALSE);

    // create display context
    window = glfwCreateWindow(windowW, windowH, "CHAI3D", NULL, NULL);
    if (!window)
    {
        cout << "failed to create window" << endl;
        cSleepMs(1000);
        glfwTerminate();
        return 1;
    }

    // set GLFW callbacks
    glfwSetKeyCallback(window, onKeyCallback);
    glfwSetWindowSizeCallback(window, onWindowSizeCallback);
    glfwSetFramebufferSizeCallback(window, onFrameBufferSizeCallback);
    glfwSetWindowContentScaleCallback(window, onWindowContentScaleCallback);

    // get width and height of window
    glfwGetFramebufferSize(window, &framebufferW, &framebufferH);

    // set position of window
    glfwSetWindowPos(window, x, y);

    // set window size
    glfwSetWindowSize(window, windowW, windowH);

    // set GLFW current display context
    glfwMakeContextCurrent(window);

    // set GLFW swap interval for the current display context
    glfwSwapInterval(swapInterval);

#ifdef GLEW_VERSION
    // initialize GLEW library
    if (glewInit() != GLEW_OK)
    {
        cout << "failed to initialize GLEW library" << endl;
        glfwTerminate();
        return 1;
    }
#endif


    //--------------------------------------------------------------------------
    // WORLD - CAMERA - LIGHTING
    //--------------------------------------------------------------------------

    // create a new world.
    world = new cWorld();

    // set the background color of the environment
    world->m_backgroundColor.setWhite();

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // The model is drawn in the YZ plane of the world: model X maps to world Y
    // (right), model Y to world Z (up). The camera looks along -X.
    camera->set(cVector3d(0.9, avatarHomeX * gfxScale, camTargetY * gfxScale),
                cVector3d(0.0, avatarHomeX * gfxScale, camTargetY * gfxScale),
                cVector3d(0.0, 0.0, 1.0));

    // set the near and far clipping planes of the camera
    camera->setClippingPlanes(0.01, 10.0);

    // set vertical mirrored display mode
    camera->setMirrorVertical(mirroredDisplay);

    // create a light source
    light = new cDirectionalLight(world);
    world->addChild(light);
    light->setEnabled(true);
    light->setDir(-1.0, 0.0, -0.3);


    //--------------------------------------------------------------------------
    // HAPTIC DEVICE
    //--------------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get access to the first available haptic device found
    handler->getDevice(hapticDevice, 0);

    // retrieve information about the current haptic device
    cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();
    maxDeviceForce = hapticDeviceInfo.m_maxLinearForce;

    if (!hapticDevice->open())
    {
        cout << "Warning - haptic device could not be opened." << endl;
    }
    hapticDevice->calibrate();


    //--------------------------------------------------------------------------
    // PHYSICAL MODEL
    //--------------------------------------------------------------------------

    createModel();
    createModelShapes();


    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    font = NEW_CFONT_CALIBRI_20();

    auto makeLabel = [&]() -> cLabel*
    {
        cLabel* label = new cLabel(font);
        label->m_fontColor.setBlack();
        camera->m_frontLayer->addChild(label);
        return label;
    };
    labelRates  = makeLabel();
    labelMissed = makeLabel();
    labelPos    = makeLabel();
    labelForce  = makeLabel();
    labelAudio  = makeLabel();

    labelMissed->setLocalPos(10, 105);
    labelPos->setLocalPos(10, 80);
    labelForce->setLocalPos(10, 55);
    labelAudio->setLocalPos(10, 30);

    // create a background
    background = new cBackground();
    camera->m_backLayer->addChild(background);
    background->setCornerColors(cColorf(1.0f, 1.0f, 1.0f),
                                cColorf(1.0f, 1.0f, 1.0f),
                                cColorf(0.8f, 0.8f, 0.8f),
                                cColorf(0.8f, 0.8f, 0.8f));


    //--------------------------------------------------------------------------
    // VIEWPORT DISPLAY
    //--------------------------------------------------------------------------

    // get content scale factor
    float contentScaleW, contentScaleH;
    glfwGetWindowContentScale(window, &contentScaleW, &contentScaleH);

    // create a viewport to display the scene.
    viewport = new cViewport(camera, contentScaleW, contentScaleH);


    //--------------------------------------------------------------------------
    // AUDIO
    //--------------------------------------------------------------------------

    ma_device_config audioConfig = ma_device_config_init(ma_device_type_playback);
    audioConfig.playback.format   = ma_format_f32;
    audioConfig.playback.channels = 2;
    audioConfig.sampleRate        = audioSampleRate;
    audioConfig.periodSizeInFrames = audioPeriodFrames;
    audioConfig.dataCallback      = onAudioData;

    if (ma_device_init(NULL, &audioConfig, &audioDev) != MA_SUCCESS)
    {
        cout << "Warning - audio device failed to initialize; the model will not run." << endl;
    }
    else
    {
        audioDevInitialized = true;
        if (ma_device_start(&audioDev) != MA_SUCCESS)
        {
            cout << "Warning - audio device failed to start." << endl;
        }
    }


    //--------------------------------------------------------------------------
    // START HAPTIC SIMULATION THREAD
    //--------------------------------------------------------------------------

    // create a thread which starts the main haptics rendering loop
    hapticsThread = new cThread();
    hapticsThread->start(renderHaptics, CTHREAD_PRIORITY_HAPTICS);

    // setup callback when application exits
    atexit(close);


    //--------------------------------------------------------------------------
    // MAIN GRAPHIC LOOP
    //--------------------------------------------------------------------------

    while (!glfwWindowShouldClose(window))
    {
        // render graphics
        renderGraphics();

        // process events
        glfwPollEvents();
    }

    // close window
    glfwDestroyWindow(window);

    // terminate GLFW library
    glfwTerminate();

    // exit
    return 0;
}

//------------------------------------------------------------------------------

void onWindowSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
    // update window size
    windowW = a_width;
    windowH = a_height;

    // render scene
    renderGraphics();
}

//------------------------------------------------------------------------------

void onFrameBufferSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
    // update frame buffer size
    framebufferW = a_width;
    framebufferH = a_height;
}

//------------------------------------------------------------------------------

void onWindowContentScaleCallback(GLFWwindow* a_window, float a_xscale, float a_yscale)
{
    // sanity check
    if (viewport == nullptr) { return; }

    // update window content scale factor
    viewport->setContentScale(a_xscale, a_yscale);
}

//------------------------------------------------------------------------------

void onErrorCallback(int a_error, const char* a_description)
{
    cout << "Error: " << a_description << endl;
}

//------------------------------------------------------------------------------

void onKeyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods)
{
    // filter calls that only include a key press
    if ((a_action != GLFW_PRESS) && (a_action != GLFW_REPEAT))
    {
        return;
    }

    // option - exit
    else if ((a_key == GLFW_KEY_ESCAPE) || (a_key == GLFW_KEY_Q))
    {
        glfwSetWindowShouldClose(a_window, GLFW_TRUE);
    }

    // option - re-center the avatar on the current device position
    else if (a_key == GLFW_KEY_O)
    {
        recenterRequested = true;
    }

    // option - toggle fullscreen
    else if (a_key == GLFW_KEY_F)
    {
        // toggle state variable
        fullscreen = !fullscreen;

        // get handle to monitor
        GLFWmonitor* monitor = glfwGetPrimaryMonitor();

        // get information about monitor
        const GLFWvidmode* mode = glfwGetVideoMode(monitor);

        // set fullscreen or window mode
        if (fullscreen)
        {
            glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
        }
        else
        {
            int w = 0.8 * mode->height;
            int h = 0.5 * mode->height;
            int x = 0.5 * (mode->width - w);
            int y = 0.5 * (mode->height - h);
            glfwSetWindowMonitor(window, NULL, x, y, w, h, mode->refreshRate);
        }

        // set the desired swap interval and number of samples to use for multisampling
        glfwSwapInterval(swapInterval);
        glfwWindowHint(GLFW_SAMPLES, 4);
    }

    // option - toggle vertical mirroring
    else if (a_key == GLFW_KEY_M)
    {
        mirroredDisplay = !mirroredDisplay;
        camera->setMirrorVertical(mirroredDisplay);
    }
}

//------------------------------------------------------------------------------

void close(void)
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // stop audio before the model is destroyed
    if (audioDevInitialized)
    {
        ma_device_uninit(&audioDev);
        audioDevInitialized = false;
    }

    // close haptic device
    hapticDevice->close();

    // delete resources
    delete hapticsThread;
    delete world;
    delete handler;
    delete model;
}

//------------------------------------------------------------------------------

void createModel()
{
    using miphysics::Vect3D;

    model = new miphysics::PhysicalModel(audioSampleRate);
    model->setGravity(grav);
    model->setFriction(fric);

    model->addHapticInput3D("hapticMass", Vect3D(0, -1, 0), ewmaSmooth);

    model->addGround3D("gnd0", Vect3D(0, 0, 0));
    for (int i = 0; i < nbMass; i++)
        model->addMass2DPlane("str" + to_string(i), massM, Vect3D(massDist * (i + 1), 0, 0), Vect3D(0, 0, 0));
    model->addMass3D("bigmass", massEnd, Vect3D(massDist * (nbMass + 1), 0, 0), Vect3D(-5, 0.1, 0));

    for (int i = 0; i < nbMass - 1; i++)
        model->addSpringDamper3D("sprd" + to_string(i), springL0, springK, springZ,
                                 "str" + to_string(i), "str" + to_string(i + 1));
    model->addSpringDamper3D("sprdg0", springL0, springK, springZ, "gnd0", "str0");
    model->addSpringDamper3D("sprdg1", springL0, springK, springZ, "bigmass", "str" + to_string(nbMass - 1));

    for (int i = 0; i < nbMass; i++)
        model->addContact3D("col", contactDist, contactK, contactZ, "hapticMass", "str" + to_string(i));
    model->addContact3D("col", contactDist, contactK, contactZ, "hapticMass", "bigmass");

    model->init();

    listenedMat = model->getMat(listeningPoint);
}

//------------------------------------------------------------------------------

// model (x, y, z) -> world (z, x, y) scaled
static cVector3d toWorld(const miphysics::Vect3D& a_v)
{
    return cVector3d(a_v.z * gfxScale, a_v.x * gfxScale, a_v.y * gfxScale);
}

void createModelShapes()
{
    for (int i = 0; i < model->getNumberOfMats(); i++)
    {
        const miphysics::Mat* mat = model->getMatAt(i);
        double radius = 0.0;
        cColorf color;
        bool transparent = false;

        switch (mat->type())
        {
        case miphysics::MatType::Mass3D:        radius = 0.3;  color.setRed();                        break;
        case miphysics::MatType::Mass2DPlane:   radius = 0.06; color.set(0.47f, 0.0f, 0.47f);          break;
        case miphysics::MatType::Ground3D:      radius = 0.3;  color.set(0.12f, 0.39f, 0.39f);         break;
        case miphysics::MatType::HapticInput3D: radius = contactDist; color.set(1.0f, 0.04f, 0.04f);   transparent = true; break;
        default:                                radius = 0.1;  color.setGrayLevel(0.5f);               break;
        }

        cShapeSphere* sphere = new cShapeSphere(radius * gfxScale);
        sphere->m_material->setColor(color);
        if (transparent) sphere->setTransparencyLevel(0.4f);
        world->addChild(sphere);
        matShapes.push_back(sphere);
    }

    for (int i = 0; i < model->getNumberOfLinks(); i++)
    {
        if (model->getLinkAt(i)->type() != miphysics::LinkType::SpringDamper3D)
        {
            linkShapes.push_back(nullptr);
            continue;
        }
        cShapeLine* line = new cShapeLine(cVector3d(0, 0, 0), cVector3d(0, 0, 0));
        line->m_colorPointA.set(0.0f, 0.0f, 1.0f);
        line->m_colorPointB.set(0.0f, 0.0f, 1.0f);
        line->setLineWidth(2.0);
        world->addChild(line);
        linkShapes.push_back(line);
    }

    updateModelShapes();
}

//------------------------------------------------------------------------------

void updateModelShapes()
{
    // positions are read under the model lock; audio blocks only for this copy
    lock_guard<mutex> lock(model->mutex());

    for (int i = 0; i < model->getNumberOfMats(); i++)
        matShapes[i]->setLocalPos(toWorld(model->getMatAt(i)->pos));

    for (int i = 0; i < model->getNumberOfLinks(); i++)
    {
        if (linkShapes[i] == nullptr) continue;
        const miphysics::Link* link = model->getLinkAt(i);
        linkShapes[i]->m_pointA = toWorld(link->mat1()->pos);
        linkShapes[i]->m_pointB = toWorld(link->mat2()->pos);
    }
}

//------------------------------------------------------------------------------

void onAudioData(ma_device* a_device, void* a_output, const void* a_input, ma_uint32 a_frames)
{
    float* out = (float*)a_output;
    const ma_uint32 channels = a_device->playback.channels;

    // hold the lock for the whole buffer
    lock_guard<mutex> lock(model->mutex());

    for (ma_uint32 i = 0; i < a_frames; i++)
    {
        model->computeStepUnlocked();

        double sample = listenedMat->pos.y * 2.0;

        // high-pass filter to remove the DC offset
        audioOut = sample - prevSample + 0.95 * audioOut;
        prevSample = sample;

        float v = (float)cClamp(audioOut, -1.0, 1.0);
        for (ma_uint32 c = 0; c < channels; c++)
            out[i * channels + c] = v;
        currAudio = v;
    }
}

//------------------------------------------------------------------------------

void renderGraphics(void)
{
    // sanity check
    if (viewport == nullptr) { return; }

    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////

    // get width and height of CHAI3D internal rendering buffer
    int displayW = viewport->getDisplayWidth();

    // update haptic and graphic rate data
    labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " +
        cStr(freqCounterHaptics.getFrequency(), 0) + " Hz");
    labelRates->setLocalPos((int)(0.5 * (displayW - labelRates->getWidth())), 15);

    labelMissed->setText("Steps Missed: " + to_string(stepsMissed.load()));
    labelPos->setText("Pos: " + cStr(posEEx.load(), 2) + ", " + cStr(posEEy.load(), 2));
    labelForce->setText("Frc: " + cStr(fEEx.load(), 4) + ", " + cStr(fEEy.load(), 4));
    labelAudio->setText("Curr Audio: " + cStr(currAudio.load(), 4));

    updateModelShapes();


    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE
    /////////////////////////////////////////////////////////////////////

    // render world
    viewport->renderView(framebufferW, framebufferH);

    // wait until all GL commands are completed
    glFinish();

    // check for any OpenGL errors
    GLenum error = glGetError();
    if (error != GL_NO_ERROR) cout << "Error: " << gluErrorString(error) << endl;

    // swap buffers
    glfwSwapBuffers(window);

    // signal frequency counter
    freqCounterGraphics.signal(1);
}

//------------------------------------------------------------------------------

void renderHaptics(void)
{
    // reset clock
    cPrecisionClock clock;
    clock.reset();

    // simulation in now running
    simulationRunning  = true;
    simulationFinished = false;

    const double nominalInterval = 1.0 / hapticRate;

    // main haptic simulation loop
    while (simulationRunning)
    {
        // stop the simulation clock
        clock.stop();

        // read the time increment in seconds
        double timeInterval = clock.getCurrentTimeSeconds();

        // restart the simulation clock
        clock.reset();
        clock.start();

        if (timeInterval > 1.5 * nominalInterval) stepsMissed++;

        // signal frequency counter
        freqCounterHaptics.signal(1);


        /////////////////////////////////////////////////////////////////////
        // HAPTIC FORCE COMPUTATION
        /////////////////////////////////////////////////////////////////////

        // device axes: X toward the user, Y right, Z up
        cVector3d devPos(0, 0, 0);
        hapticDevice->getPosition(devPos);

        if (recenterRequested.exchange(false))
        {
            offsetX = avatarHomeX - alpha * devPos.y();
            offsetY = avatarHomeY - alpha * devPos.z();
        }

        double mx = alpha * devPos.y() + offsetX;
        double my = alpha * devPos.z() + offsetY;
        posEEx = mx;
        posEEy = my;

        // set the position of the avatar, and fetch the accumulated force
        model->setHapticPosition("hapticMass", miphysics::Vect3D(mx, my, 0.0));
        miphysics::Vect3D frc = model->getHapticForce("hapticMass");

        cVector3d force(0.0, beta * frc.x, beta * frc.y);
        if (force.length() > maxDeviceForce)
        {
            force.normalize();
            force.mul(maxDeviceForce);
        }
        fEEx = force.y();
        fEEy = force.z();

        // send force to haptic device
        hapticDevice->setForce(force);
    }

    // release the device
    hapticDevice->setForce(cVector3d(0, 0, 0));

    // exit haptics thread
    simulationFinished = true;
}

//------------------------------------------------------------------------------
