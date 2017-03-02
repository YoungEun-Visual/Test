#ifndef EyeTracker__
#define EyeTracker__

#include "VCParams.h"

#include "utils/VCFilter.h"
#include "ofMain.h"
#include "ofxOpenCv.h"
#include <stdlib.h>
#include <string>
#include <vector>
#include "imageProc/PupilDetector.h"
#include "Calibration.h"
#include "AlgorithmEvaluation.h"
#include "imageProc/DominantMovement.h"

#if APP_TYPE == APP_TYPE_OFX_NORMAL
    #include "ofxGui.h"
#endif
#if THREAD_PTHREAD
    #include <pthread.h>
#endif

#if OUTPUT_ANDROID
    #include "ofxAndroid.h"
    #include <jni.h>
    #include "utils/VCAndroidTool.h"
#endif

#if VIDEO_RECORDING
    #include "../VideoSet.h"
#endif

#if CAMERA_LIBUVC
    #include "utils/UVCCamera.h"
#endif

#define MAX_NUMBER_GAZEPOINT 80
#define MIN_NUMBER_LIMIT_GAZEPOINT 40

#define NUMBER_CALIBRATION_POINTS 9
#define CALIBRATION_ANIMATION_TIME 400

#define NUMBER_PUPIL_HISTORY    FPS_PROCESSING / 2
#define INTERVAL_CHECK_POSITION 2



// Set Image Buffer Size and Image Number
// prefix I for IplImage Numbers
// G : Gray Image
// C : Color Image
// SRC : Original Eye Images
// REF : Images for Processing
// PRT : Images for Printing
// HIST : Images for histogram(w = 256)
#if PRINT_IMAGES
    #define I_ARR_N             13

    #define I_GSRC              0
    #define I_CSRC              1

    #define I_GPRT_0            2
    #define I_GPRT_1            3
    #define I_GPRT_2            4
    #define I_GPRT_3            5
    #define I_GPRT_4            6
    #define I_GPRT_5            7

    #define I_GREF_0            8

    #define I_CPRT_0            9
    #define I_CPRT_1            10

    #define I_CHIST_0           11
    #define I_CHIST_1           12


#elif PRINT_IMAGES_MINIMALIZE
    #define I_ARR_N             3

    #define I_CSRC              0
    #define I_GSRC              1
    #define I_GPRT_0            2


#else
    #define I_ARR_N             2

    #define I_GSRC              0
    #define I_CSRC              1

#endif




#define CV_USE_VIDEO            0
#define CV_USE_LIVE_VIDEO       1
#define CV_USE_PICTURE          2


#if (VIDEO_OFX | CAMERA_OFX)
    typedef struct {
        int mode;
        ofVideoGrabber vg;
        ofVideoPlayer  vp;
        bool ready;
    } video_context;

#endif


#if APP_TYPE == APP_TYPE_CUSTOM_ANDROID
class EyeTracker {
    
#elif APP_TYPE == APP_TYPE_OFX_ANDROID
class EyeTracker : public ofxAndroidApp {
    
#elif APP_TYPE == APP_TYPE_OFX_NORMAL
class EyeTracker : public ofBaseApp {
    
#endif

private:
    const long calibrationCheckTime = 2000;
    const float pointsIncludingGazePointFixed = 0.8;
    const float circleSizeIncluding = 160;
    const float circleSizeFailLimit = 1800;

    bool cameraFrameNew;
    bool justPass;

    int d_circle_cntx, d_circle_cnty, d_circle_radius;
    bool is_center_dotted=false, is_circle_exist=false;
    
    int d_rect_pt1x, d_rect_pt1y, d_rect_pt2x, d_rect_pt2y;
    bool is_first_pt_dotted = false, is_rect_exist = false;
    
    bool wait_set_threshold, setting_manual_threshold, manual_threshold_fixed, need_refresh, pupil_found;
    int manualThrshold;
    
    void printMask();
    
public:
    EyeTracker();

    void setup();
    void update();
    
    void init(int *dstPoint);
    void initFrameBuffer( CvSize the_size );
    int initCamera(int vid=CAMERA_VID, int pid=CAMERA_PID, int fd=0, const char *fs="");
    
    void terminate();
    
    void secondECPro();
    void gazePointProcess();
    void catchDominantMoveProcess();
    bool getFrame( IplImage *ipl_csrc );
    bool makeGrayImage(IplImage *src, IplImage *dst);
    bool makeImageMask(IplImage *img);
    void moveImgAndSubstract(IplImage *src, IplImage *base, IplImage *dst, IplImage *tmp, int xdiff, int ydiff);

//    #if NEED_CERTIFICATION
        bool isValidApplication = false;

//    #endif

    #if USE_KEYBOARD_INPUT
        void keyPressed(int key);
        void keyReleased(int key);
        void mouseMoved(int x, int y );
        void mouseDragged(int x, int y, int button);
        void mousePressed(int x, int y, int button);
        void mouseReleased(int x, int y, int button);
    
    #endif
    
    
    #if APP_TYPE == APP_TYPE_OFX_NORMAL
        ofxPanel gui;
        ofxToggle isSettingRoi;
        ofxToggle settingThreshold;
    
    
        void windowResized(int w, int h);
        void dragEvent(ofDragInfo dragInfo);
        void gotMessage(ofMessage msg);
        #if VIDEO_OFX
            void settingToggleCallback(bool & setting);
        #endif

    #endif
    
    
    
    #if PRINT_IMAGES_MINIMALIZE
        ofImage                 img_debugging_out1;
        ofImage                 img_debugging_out2;

    #endif
    
    
    #if PRINT_IMAGES
        ofImage                 img_debugging_out1;
        ofImage                 img_debugging_out2;
        ofImage                 img_debugging_out3;
        ofImage                 img_debugging_out4;
        ofImage                 img_debugging_out5;
        ofImage                 img_debugging_out6;
        
        ofxCvColorImage         img_color_out0;
        ofxCvColorImage         img_color_out1;
        ofxCvColorImage         img_color_out2;

    #endif

    
    
    bool isInitialized;
    bool printingDebug = true;
    void touchDown(int x, int y, int id);

    IplImage *ipl_arr[I_ARR_N];

    VCFilter *_gfilter;
    PupilDetector detector;
    
    CvSize          cap_size;
    CvBox2D         pupil_fit_cur;
    CvPoint2D32f	guess_pt;
    CvPoint2D32f    screen_pt_raw;
    CvPoint2D32f	screen_pt_avg;
    
    
    #if USE_CALIBRATION_PROCESS // for Andorid whole calibration parts
        
        Calibration *calibrationData;
        
        void setRecentCalibrationInfo(float* srcPoint, float* dstPoint);
        void initCalibrationProcess();
        bool CalibrationProcess();
        bool isCalibrationPointFixed(ofVec2f &result, ofVec2f* history, int numberOfPoints);
        bool isCalibrationPointFixedRecursive(ofVec2f &result, ofVec2f* history, int numberOfPoints, int numberOfCurrentPoints);
        
        ofVec2f gazeHistory[MAX_NUMBER_GAZEPOINT];
        CvPoint calibrationOrder[NUMBER_CALIBRATION_POINTS];
        
        int currentNumberOfGazePoint;
        long calibrationTime;
        bool bCalibrationProcessDone;
        int curCalibrationStep;
        VCFilter *calibrationFilter;
        
    #endif

    
    
    #if VIDEO_RECORDING
        VideoSet* vs;
        bool bVideoReady;
        bool bStartRec;
        bool bRecCurDisplay;
        bool bRecCurDisplayforTest;
        
        ofImage img_window_capture;
        
    #endif
    
    
    
    #if (VIDEO_OFX | CAMERA_OFX)
        video_context cap_context;
        bool bVideoStopped;
        bool initVideo( video_context *vid_context,CvSize *size, int iCamNum);
        
        int videoStep;
        float playSpeed;
        
    #endif

    
    
    #if THREAD_PTHREAD
        pthread_mutex_t camera_mutex, process_mutex;
        pthread_cond_t camera_cond, calibration_cond;
        pthread_mutex_t dm_mutex;
        pthread_cond_t dm_cond;
    #endif
    
    
    
    #if COMPARE_ALGORITHM
        PupilDetector compareDetector;
        pupilDetectionAlgorithm compareAlgorithm;
        CvBox2D pupil_fit_cur_compare;
        
    #endif



    #if (PRINT_FRAMERATE | PRINT_IMAGES | PRINT_IMAGES_MINIMALIZE)
        long long timer_start;
        long processingTime;
        float processingRatio;
        int frameCount;
        float fps;
    
    #endif
    
    

    #if DOMINANT_MOVE
        VCFilter translation_Filter1;
        DominantMovement *movement;
        bool capture, check;
        CvPoint2D32f translation;
        int captureCount;
        #if THREAD_PTHREAD
            void start_dm_thread();
        #endif

    #endif
    
    
    
    #if (PRINT_IMAGES | PRINT_IMAGES_MINIMALIZE)
        void draw();
        void drawStatusCheck();
        void drawLetterBoard();
        void setHistImg(float* hist, IplImage* dest, int thres, int thres2 = 0);
        void setHistImg(float* hist, IplImage* dest, vector<int> peak, vector<int> valley);
        void setBottomHist(float* hist, IplImage* dest, vector<int> peak, vector<int> valley);
        void setBottomHist(float* hist, IplImage* dest, int thres, int thres2 = 0);

        float hist_iris[256], hist_deriv[256];
        char reportStr[1024];
        ofTrueTypeFont bigFont0;
        vector<int> peakList, valleyList;
    
    #endif
    
    
    #if CAMERA_LIBUVC
        UVCCamera *uvcCamera;
    #endif
    
    #if OUTPUT_ANDROID
        void getSignature(JNIEnv *env, jobject context);
    #endif
};

#endif // ifdef EyeTracker__
