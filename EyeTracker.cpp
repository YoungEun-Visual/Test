#include "EyeTracker.h"

#include <stdio.h>
#include <math.h>
#include "utils/connection.h"

#define FPS_HI 60
#define FPS_LO 30

#define CAM_RES_X 640
#define CAM_RES_Y 480
#define PUPIL_ANGLE_FACTOR 15

#define CARIBRATION_POINT_DELAY 2.0
#define PUPIL_SIZE_LIMIT 0.3

#define MANUAL_THRESHOLD 45
#define WHITE_THRESHOLD 180

#define JNI_TRUE 1
#define JNI_FALSE 0


const char server_ip[30] = "203.233.111.50";
int portNum = 2180;
#if OUTPUT_ANDROID
// Context 를 인자값을 받아서 Signature 의 값을 얻는다.
void EyeTracker::getSignature(JNIEnv *env, jobject context) {
    #if NEED_CERTIFICATION
    jstring packageName;
    jobject packageManagerObj;
    jobject packageInfoObj;
    jclass contextClass =  env->GetObjectClass( context);

    //get application apk size
    jmethodID getApplicationInfo = env->GetMethodID( contextClass, "getApplicationInfo", "()Landroid/content/pm/ApplicationInfo;");
    jclass applicationInfoClass = env->FindClass("android/content/pm/ApplicationInfo");
    jfieldID sourceDirID = env->GetFieldID(applicationInfoClass, "sourceDir", "Ljava/lang/String;");
    jobject applicationInfo = (jobject)env->CallObjectMethod(context, getApplicationInfo);
    jstring sourceDir = (jstring)env->GetObjectField(applicationInfo, sourceDirID);

    jclass fileClass = env->FindClass("java/io/File");
    jmethodID fileInit = env->GetMethodID(fileClass, "<init>", "(Ljava/lang/String;)V");
    jmethodID getLength = env->GetMethodID(fileClass, "length", "()J");
    jobject newfile = env->NewObject(fileClass, fileInit, sourceDir);
    jlong fileLength = (jlong)env->CallLongMethod(newfile, getLength);

    ofLogNotice("app size") << "app size : " << (long)fileLength;


    // get application signature id
    jmethodID getPackageNameMid =  env->GetMethodID( contextClass, "getPackageName", "()Ljava/lang/String;");
    jmethodID getPackageManager =  env->GetMethodID( contextClass, "getPackageManager", "()Landroid/content/pm/PackageManager;");


    jclass packageManagerClass = env->FindClass("android/content/pm/PackageManager");
    jmethodID getPackageInfo = env->GetMethodID( packageManagerClass, "getPackageInfo", "(Ljava/lang/String;I)Landroid/content/pm/PackageInfo;");

    jclass packageInfoClass = env->FindClass("android/content/pm/PackageInfo");
    jfieldID signaturesFid = env->GetFieldID( packageInfoClass, "signatures", "[Landroid/content/pm/Signature;");

    jclass signatureClass = env->FindClass("android/content/pm/Signature");
    jmethodID signatureToByteArrayMid = env->GetMethodID( signatureClass, "toByteArray", "()[B");

    jclass messageDigestClass = env->FindClass("java/security/MessageDigest");
    jmethodID messageDigestUpdateMid = env->GetMethodID( messageDigestClass, "update", "([B)V");
    jmethodID getMessageDigestInstanceMid  = env->GetStaticMethodID( messageDigestClass, "getInstance", "(Ljava/lang/String;)Ljava/security/MessageDigest;");
    jmethodID digestMid = env->GetMethodID( messageDigestClass,"digest", "()[B");

    jclass base64Class = env->FindClass("android/util/Base64");
    jmethodID encodeToStringMid = env->GetStaticMethodID( base64Class,"encodeToString", "([BI)Ljava/lang/String;");

    packageName =  (jstring)env->CallObjectMethod( context, getPackageNameMid);
    packageManagerObj = env->CallObjectMethod(context, getPackageManager);

    packageInfoObj = env->CallObjectMethod( packageManagerObj,getPackageInfo, packageName, 0x40);
    jobjectArray signatures = (jobjectArray)env->GetObjectField( packageInfoObj, signaturesFid);

    jobject signatureObj = env->GetObjectArrayElement(signatures, 0);
    jobject messageDigestObj  = env->CallStaticObjectMethod(messageDigestClass, getMessageDigestInstanceMid, env->NewStringUTF("SHA-256"));
    env->CallVoidMethod(messageDigestObj, messageDigestUpdateMid, env->CallObjectMethod( signatureObj,signatureToByteArrayMid));

    // Base64.DEFAULT = 0 그렇기 때문에 맨 마지막 인자값은 0이다.
    jstring signatureHash = (jstring)env->CallStaticObjectMethod( base64Class, encodeToStringMid,env->CallObjectMethod( messageDigestObj, digestMid ), 0);
    const char *name = env->GetStringUTFChars(signatureHash, NULL);

    int res = connection::send_hello_message(server_ip, portNum, (int)fileLength, name);
//    connection::generate_hello_key((int)fileLength, name);
    if (res == 1) isValidApplication = true;
    else {
        ofLogNotice("connection") << "connection failed : " << res;
    }

    ofLogNotice("SHA-256") << "SHA : " << name;
    env->ReleaseStringUTFChars(signatureHash, name);
    #else
        isValidApplication = true;
    #endif
}
#endif

int cameraFrameRate = 0;
int processingRate = 0;

#if (VIDEO_OFX | CAMERA_OFX)
video_context video_context_new(int mode) {
    assert( mode == CV_USE_VIDEO || mode == CV_USE_LIVE_VIDEO );
    video_context a;
    a.ready = false;
    a.mode = mode;
    return a;
}
#endif

EyeTracker::EyeTracker()
:   isInitialized(false){
    #if THREAD_PTHREAD
        pthread_mutex_init(&camera_mutex, NULL);
        pthread_mutex_init(&process_mutex, NULL);
        pthread_cond_init(&camera_cond, NULL);
        pthread_cond_init(&calibration_cond, NULL);

        pthread_mutex_init(&dm_mutex, NULL);
        pthread_cond_init(&dm_cond, NULL);
    
    #endif

//    #if NEED_CERTIFICATION
    isValidApplication = false;

//    #endif
}


bool EyeTracker::makeGrayImage(IplImage *src, IplImage *dst){
    int h,w;
    int gstep_size = dst->widthStep;
    int cstep_size = src->widthStep;
    

    uchar *cur_gray_ptr=0;
    uchar *cur_color_ptr = 0;

    if( cap_size.width != 640 ||
        cap_size.height != 480 )
        return false;

    for(h=0; h < cap_size.height; h++){
        cur_gray_ptr = (uchar*)dst->imageData + h*gstep_size;
        cur_color_ptr = (uchar*)src->imageData + (cap_size.height-h)*cstep_size - IMAGE_COLOR_DIM;
        //pre mask
        for(w=0; w < cap_size.width ; w++){
            *cur_gray_ptr++ = *cur_color_ptr;
                cur_color_ptr-=IMAGE_COLOR_DIM;
        }
    }
    return true;
}

bool EyeTracker::makeImageMask(IplImage *img) {
    int h, w;
    int gstep_size = img->widthStep;

    uchar *cur_gray_ptr = 0;

    if( cap_size.width != 640 ||
       cap_size.height != 480 )
        return false;

    for (h = 0; h < cap_size.height; h++) {
        cur_gray_ptr = (uchar*)img->imageData + h * gstep_size;

        //pre mask
        for (w = 0; w < MIN(ELIPSE_START_COL[h], cap_size.width); w++)
            *cur_gray_ptr++ = 255;

        for (; w <= MIN(ELIPSE_END_COL[h], cap_size.width - 1); w++) {
            *cur_gray_ptr++;
        }

        //post mask
        for (; w < cap_size.width; w++)
            *cur_gray_ptr++ = 255;
    }

    return true;
}


int EyeTracker::initCamera(int vid, int pid, int fd, const char *fs) {
#if CAMERA_LIBUVC
    return uvcCamera->openCamera(vid, pid, fd, fs);
#endif
}


void EyeTracker::init(int *dstPoint) {

    if(PRINT_DEBUG) ofLogNotice("EyeTracker") << "init";
    setup();


#if USE_CALIBRATION_PROCESS
        calibrationData = new Calibration(cvPoint2D32f(CAM_RES_X, CAM_RES_Y), 3, 3);
        calibrationData->setDstPoints(dstPoint);
    
        // set calibration order which starts at center ends at right down
        calibrationOrder[0] = cvPoint(1, 1);
        calibrationOrder[1] = cvPoint(2, 1);
        calibrationOrder[2] = cvPoint(2, 0);
        calibrationOrder[3] = cvPoint(1, 0);
        calibrationOrder[4] = cvPoint(0, 0);
        calibrationOrder[5] = cvPoint(0, 1);
        calibrationOrder[6] = cvPoint(0, 2);
        calibrationOrder[7] = cvPoint(1, 2);
        calibrationOrder[8] = cvPoint(2, 2);

        screen_pt_raw = cvPoint2D32f(0,0);
        screen_pt_avg = cvPoint2D32f(0, 0);
        calibrationFilter = new VCFilter();
        calibrationFilter->SetFilter(VCFilter::ONE_EURO);
    #endif
}


void EyeTracker::setup(){
	if(PRINT_DEBUG)ofLogNotice("EyeTracker") << "setup:";

    isInitialized = true;
    
    _gfilter = new VCFilter();
    _gfilter->SetFilter(VCFilter::ONE_EURO);
    
    cap_size = cvSize(CAM_RES_X, CAM_RES_Y);
    initFrameBuffer( cap_size );

    detector.setSourceImage(ipl_arr[I_GSRC]);
    detector.setAlgorithm(ROBUST_AND_NEAR_PAST);
    detector.setRobustAlgorithm(ALGORITHM_COMPARE_HISTOGRAM);
    
    justPass = false;
    pupil_fit_cur.center = cvPoint2D32f(cap_size.width*0.5, cap_size.height*0.5);
    pupil_fit_cur.size   = cvSize2D32f(35, 35);
    guess_pt = cvPoint2D32f(0, 0);

    
    #if (PRINT_FRAMERATE | PRINT_IMAGES | PRINT_IMAGES_MINIMALIZE)
        if (PRINT_DEBUG) ofLogNotice("EyeTracker") << "setup: frame rate / processing time";
        timer_start = ofGetElapsedTimeMillis();
        processingTime = 0;
        processingRatio = 0.0f;
        frameCount = 0;
        fps = 0.0f;
        
    #endif

    #if DOMINANT_MOVE
        if (PRINT_DEBUG) ofLogNotice("EyeTracker") << "setup: dominant move";
        translation = cvPoint2D32f(0, 0);
        #if THREAD_PTHREAD
            movement = new DominantMovement(&dm_mutex, &dm_cond);
        #else
            movement = new DominantMovement();
        #endif
        movement->setCompareImage(ipl_arr[I_GSRC]);

        capture = false;
        captureCount = 0;
        translation_Filter1.SetFilter(VCFilter::ONE_EURO);

    #endif

    #if (VIDEO_OFX | CAMERA_OFX)
        if (PRINT_DEBUG) ofLogNotice("EyeTracker") << "setup: ofxvideo";
        int EYE_CAM_ID      = 0;
        videoStep           = 0;
        playSpeed           = 1.0;
        bVideoStopped       = false;
    
        cap_context = video_context_new( CV_USE_VIDEO );
//        cap_context = video_context_new( CV_USE_LIVE_VIDEO );
    
        bVideoReady = initVideo( &cap_context, &cap_size , EYE_CAM_ID);//Eye Cam
        assert( bVideoReady );
    
    #endif
    
    
    #if VIDEO_RECORDING
        if (PRINT_DEBUG) ofLogNotice("EyeTracker") << "setup: video recording";
        bStartRec           = false;
        bRecCurDisplay      = false;
        bRecCurDisplayforTest = false;
        vs = new VideoSet();

    #endif
    
    
    #if APP_TYPE == APP_TYPE_OFX_NORMAL
        if (PRINT_DEBUG) ofLogNotice("EyeTracker") << "setup: normal ofx app / window and frame rate setting";
        int SCREEN_SIZE_X = 1024;
        int SCREEN_SIZE_Y = 768;
        ofSetWindowShape(SCREEN_SIZE_X, SCREEN_SIZE_Y);
        ofSetFullscreen(false);
		ofSetFrameRate(FPS_HI);
        wait_set_threshold = false;
        setting_manual_threshold = false;
        manual_threshold_fixed = false;
        manualThrshold = 0;

        #if VIDEO_OFX
            settingThreshold.addListener(this, &EyeTracker::settingToggleCallback);
        #endif
    
        gui.setup();
        gui.add(isSettingRoi.setup("set roi", false));
        gui.add(settingThreshold.setup("setting threshold", false));
    
    
        #if COMPARE_ALGORITHM
            if (PRINT_DEBUG) ofLogNotice("EyeTracker") << "setup: compare algorithm";
            compareAlgorithm = ROBUST_AND_NEAR_PAST;
            compareDetector.setSourceImage(ipl_arr[I_GSRC]);
            compareDetector.setAlogirhtm(compareAlgorithm);
            compareDetector.setRobustAlogirhtm(ALGORITHM_COMPARE_HISTOGRAM);

        #endif

    #endif
    

    #if (PRINT_IMAGES | PRINT_IMAGES_MINIMALIZE)
        if (PRINT_DEBUG) ofLogNotice("EyeTracker") << "setup: hist_iris / font";
        for(int i=0;i<256;i++){
            hist_iris[i] = -1;
        }
        bigFont0.loadFont("EyeDoctor.otf", 20, true);
    
    #endif
    
    
    #if CAMERA_LIBUVC
        if (PRINT_DEBUG) ofLogNotice("EyeTracker") << "setup: uvc camera";
        uvcCamera = new UVCCamera(cap_size, 30, ipl_arr[I_GSRC]->imageData);
        #if APP_TYPE != APP_TYPE_CUSTOM_ANDROID
            initCamera();
        #endif
    
	#endif

}




void EyeTracker::initFrameBuffer( CvSize the_size ){
    if (PRINT_DEBUG) ofLogNotice("EyeTracker") << "initFrameBuffer:";

    // allocate OpenCV IplImage "registers" for various tasks
    // default to grayscale

    for( int i=0; i<I_ARR_N; i++ ){
        int arr_w = the_size.width;
        int arr_h = the_size.height;
        int arr_d = IPL_DEPTH_8U;
        int arr_c = 1;
        
        if(i==I_CSRC) { arr_c = IMAGE_COLOR_DIM; }
        
        #if PRINT_IMAGES
            if( i==I_CPRT_1 ) { arr_c = IMAGE_COLOR_DIM; }
            if( i==I_CHIST_0 ) { arr_c = IMAGE_COLOR_DIM; arr_w = 256;}
            if( i==I_CHIST_1 ) { arr_c = IMAGE_COLOR_DIM; arr_w = 256;}
            if( i==I_CPRT_0  ) { arr_c = 3; }
        
        #endif

        ipl_arr[i] = cvCreateImage(cvSize(arr_w,arr_h), arr_d, arr_c);
        cvSetZero(ipl_arr[i]);
    }

    #if PRINT_IMAGES_MINIMALIZE
        if (PRINT_DEBUG) ofLogNotice("EyeTracker") << "initFrameBuffer: buffer allocation for minimalize output";
        img_debugging_out1.allocate(cap_size.width, cap_size.height, OF_IMAGE_GRAYSCALE);
        img_debugging_out2.allocate(cap_size.width, cap_size.height, OF_IMAGE_GRAYSCALE);

    #endif
    
    
    #if PRINT_IMAGES
        if (PRINT_DEBUG) ofLogNotice("EyeTracker") << "initFrameBuffer: buffer allocation for image output";
        img_color_out0.allocate(cap_size.width, cap_size.height);
        img_color_out1.allocate(256, cap_size.height);
        img_color_out2.allocate(256, cap_size.height);
    
        img_debugging_out1.allocate(cap_size.width, cap_size.height, OF_IMAGE_GRAYSCALE);
        img_debugging_out2.allocate(cap_size.width, cap_size.height, OF_IMAGE_GRAYSCALE);
        img_debugging_out3.allocate(cap_size.width, cap_size.height, OF_IMAGE_GRAYSCALE);
        img_debugging_out4.allocate(cap_size.width, cap_size.height, OF_IMAGE_GRAYSCALE);
        img_debugging_out5.allocate(cap_size.width, cap_size.height, OF_IMAGE_GRAYSCALE);
        img_debugging_out6.allocate(cap_size.width, cap_size.height, OF_IMAGE_GRAYSCALE);
    
    #endif
    
    
    #if VIDEO_RECORDING
        if (PRINT_DEBUG) ofLogNotice("EyeTracker") << "initFrameBuffer: buffer allocation for recording";
        img_window_capture.allocate(ofGetWindowWidth(), ofGetWindowHeight(), OF_IMAGE_COLOR);
    
    #endif
}



void EyeTracker::update(){
    if (LIKELY(isInitialized)) {
        if (PRINT_SETFRAME) ofLogNotice("EyeTracker") << "update";
        
        LOCK_THREAD_WITH_MUTEX(&process_mutex)
        if (LIKELY(isInitialized)) {
            //#1 Find Eye center point from Eye Camera Image
            long starttime = ofGetElapsedTimeMillis();
            secondECPro();
            
        }
        UNLOCK_THREAD_WITH_MUTEX(&process_mutex)
    }
}


void EyeTracker::secondECPro() {
    #if APP_TYPE == APP_TYPE_OFX_NORMAL
    #endif

    #if NEED_CERTIFICATION
        if(UNLIKELY(!isValidApplication)) return;

    #endif
    if (PRINT_SETFRAME) ofLogNotice("EyeTracker") << "secondECPro";
    pupil_found = false;
    cameraFrameNew = getFrame(ipl_arr[I_CSRC]);
    
    if (cameraFrameNew || justPass) {
        manual_threshold_fixed = false;
        justPass = false;
        long processingStartTime = ofGetElapsedTimeMillis();
        
        #if CAMERA_OFX
            LOCK_THREAD_WITH_MUTEX(&dm_mutex);
            makeGrayImage(ipl_arr[I_CSRC], ipl_arr[I_GSRC]);
            #if DOMINANT_MOVE && PRINT_IMAGES
                if(capture){
                    cvCopy(ipl_arr[I_GSRC], ipl_arr[I_GREF_0]);
                }
            #endif
        
            UNLOCK_THREAD_WITH_MUTEX(&dm_mutex);
        
        #endif

        
        #if USE_MASK
            LOCK_THREAD_WITH_MUTEX(&dm_mutex);
            makeImageMask(ipl_arr[I_GSRC]);
            UNLOCK_THREAD_WITH_MUTEX(&dm_mutex);

        #endif

        #if CAMERA_OFX
            int currFrame = cap_context.vp.getCurrentFrame();
            detector.setFrameNew(currFrame);
        #else
            detector.setFrameNew();
        #endif
        LOCK_THREAD_WITH_MUTEX(&dm_mutex);
        pupil_found = detector.getPupilPosition(&pupil_fit_cur);
        #if THREAD_PTHREAD
            pthread_cond_signal(&dm_cond);
        #endif
        UNLOCK_THREAD_WITH_MUTEX(&dm_mutex);

        #if COMPARE_ALGORITHM
            compareDetector.setFrameNew(currFrame);
            LOCK_THREAD_WITH_MUTEX(&dm_mutex);
            compareDetector.getPupilPosition(&pupil_fit_cur_compare);
            UNLOCK_THREAD_WITH_MUTEX(&dm_mutex);

        #endif
        
        if(pupil_found){
            catchDominantMoveProcess();
        }


        
        #if (PRINT_FRAMERATE | PRINT_IMAGES | PRINT_IMAGES_MINIMALIZE)
            processingTime += ofGetElapsedTimeMillis() - processingStartTime;
            frameCount ++;
            if (ofGetElapsedTimeMillis() - timer_start >= 1 * 1000){
                fps = frameCount;
                frameCount = 0;
                timer_start = ofGetElapsedTimeMillis();
                processingRatio = processingTime / 1000.0f;
                processingTime = 0;

                #if PRINT_FRAMERATE
                    ofLogNotice("EyeTracker") << "fps : " << fps;
                    ofLogNotice("EyeTracker") << "processingRatio : " << processingRatio;
                
                #endif
            }
        
        #endif
    }
    if (need_refresh){
        LOCK_THREAD_WITH_MUTEX(&dm_mutex);
        pupil_found = detector.getPupilPositionSameCond(&pupil_fit_cur, manualThrshold);
        need_refresh = false;
        UNLOCK_THREAD_WITH_MUTEX(&dm_mutex);
    }
    gazePointProcess();

}

#if DOMINANT_MOVE && THREAD_PTHREAD
void EyeTracker::start_dm_thread() {
    movement->start_dominant_move_thread(&pupil_fit_cur, &translation);
}
#endif

void EyeTracker::catchDominantMoveProcess(){
#if DOMINANT_MOVE
    if(capture){
        captureCount++;
        capture = false;
        if (captureCount == 1){
            movement->setReferenceImage(ipl_arr[I_GSRC], pupil_fit_cur);
        }

    }
    #if !THREAD_PTHREAD
        if (captureCount > 0){
            CvPoint2D32f tempTrans = translation;
            bool changed = movement->getTranslation(ipl_arr[I_GSRC], &tempTrans, pupil_fit_cur, translation);
            if (changed && (tempTrans.x != translation.x || tempTrans.y != translation.y)){
                if(PRINT_DOMINANTMOVE) ofLog() << "translation by LK    : " << tempTrans.x << ", " << tempTrans.y;

                translation_Filter1.Filter(tempTrans.x, tempTrans.y);
                translation.x = tempTrans.x;
                translation.y = tempTrans.y;
            }
        }
    #endif
#endif
}

void EyeTracker::gazePointProcess() {
    #if OUTPUT_ANDROID
        CvPoint2D32f temp = cvPoint2D32f(0, 0);
        #if DOMINANT_MOVE
            LOCK_THREAD_WITH_MUTEX(&dm_mutex);
            temp = translation;
            UNLOCK_THREAD_WITH_MUTEX(&dm_mutex);

        #endif
        LOCK_THREAD_WITH_MUTEX(&dm_mutex);
        guess_pt.x = pupil_fit_cur.center.x + temp.x;
        guess_pt.y = pupil_fit_cur.center.y + temp.y;
        calibrationData->queryCalibrationMap(guess_pt, &screen_pt_raw);

        float filtered_x = screen_pt_raw.x;
        float filtered_y = screen_pt_raw.y;

        _gfilter->Filter(filtered_x, filtered_y);
        screen_pt_avg = cvPoint2D32f(filtered_x, filtered_y);

        int res[9];
        res[0] = screen_pt_avg.x;
        res[1] = screen_pt_avg.y;
        res[2] = cap_size.width;
        res[3] = cap_size.height;
        res[4] = pupil_fit_cur.center.x + temp.x;
        res[5] = pupil_fit_cur.center.y + temp.y;
        res[6] = pupil_fit_cur.size.width*0.5;
        res[7] = pupil_fit_cur.size.height*0.5;
        res[8] = pupil_fit_cur.angle;
        UNLOCK_THREAD_WITH_MUTEX(&dm_mutex);
        VCAndroidTool::getInstance().setGazePointInfo(res, 9);
        VCAndroidTool::getInstance().doGazePointCallback();

    #elif PRINT_IMAGES_MINIMALIZE
        if (printingDebug) {
            CvPoint2D32f temp = cvPoint2D32f(0, 0);
            
            #if DOMINANT_MOVE
                LOCK_THREAD_WITH_MUTEX(&dm_mutex);
                temp = translation;
                UNLOCK_THREAD_WITH_MUTEX(&dm_mutex);
                moveImgAndSubstract(ipl_arr[I_GSRC], NULL, NULL, ipl_arr[I_GPRT_0], -temp.x, -temp.y);
            
            #else
                cvCopy(ipl_arr[I_GSRC], ipl_arr[I_GPRT_0]);
            
            #endif
            
            LOCK_THREAD_WITH_MUTEX(&dm_mutex);
            cvEllipse(ipl_arr[I_GPRT_0],
                      cvPoint(pupil_fit_cur.center.x + temp.x, pupil_fit_cur.center.y + temp.y),
                      cvSize(pupil_fit_cur.size.width * 0.5, pupil_fit_cur.size.height * 0.5),
                      pupil_fit_cur.angle, 0, 360.0, CV_RGB(255, 255, 255));
            cvCircle(ipl_arr[I_GPRT_0], cvPoint(pupil_fit_cur.center.x + temp.x, pupil_fit_cur.center.y + temp.y), 4,
                     CV_RGB(255, 255, 255));


            img_debugging_out1.setFromPixels((uchar *) ipl_arr[I_GSRC]->imageData, cap_size.width, cap_size.height, OF_IMAGE_GRAYSCALE);
            UNLOCK_THREAD_WITH_MUTEX(&dm_mutex);
            img_debugging_out2.setFromPixels((uchar *) ipl_arr[I_GPRT_0]->imageData, cap_size.width, cap_size.height, OF_IMAGE_GRAYSCALE);
        }
    
    #elif PRINT_IMAGES
        LOCK_THREAD_WITH_MUTEX(&dm_mutex);
        cvCvtColor(ipl_arr[I_GSRC], ipl_arr[I_CPRT_0], CV_GRAY2BGR);

        #if COMPARE_ALGORITHM
            float compare_angle = pupil_fit_cur_compare.angle * PI / 180;
            float compare_width = pupil_fit_cur_compare.size.width*0.5;
            float compare_height = pupil_fit_cur_compare.size.height*0.5;
            cvLine(ipl_arr[I_CPRT_0],
                   cvPoint(pupil_fit_cur_compare.center.x - cos(compare_angle) * compare_width, pupil_fit_cur_compare.center.y - sin(compare_angle) * compare_width),
                   cvPoint(pupil_fit_cur_compare.center.x + cos(compare_angle) * compare_width, pupil_fit_cur_compare.center.y + sin(compare_angle) * compare_width),
                   CV_RGB(255, 150, 0), 1, CV_AA);
            cvLine(ipl_arr[I_CPRT_0],
                   cvPoint(pupil_fit_cur_compare.center.x + sin(compare_angle) * compare_height, pupil_fit_cur_compare.center.y - cos(compare_angle) * compare_height),
                   cvPoint(pupil_fit_cur_compare.center.x - sin(compare_angle) * compare_height, pupil_fit_cur_compare.center.y + cos(compare_angle) * compare_height),
                   CV_RGB(255, 255, 0), 1, CV_AA);
            
            cvEllipse(ipl_arr[I_CPRT_0],
                      cvPoint(pupil_fit_cur_compare.center.x, pupil_fit_cur_compare.center.y),
                      cvSize(compare_width, compare_height),
                      pupil_fit_cur_compare.angle, 0, 360.0, CV_RGB(255, 255, 0), 1, CV_AA);
            cvCircle(ipl_arr[I_CPRT_0], cvPoint(pupil_fit_cur_compare.center.x, pupil_fit_cur_compare.center.y), 4, CV_RGB(255, 255, 0), 3, CV_AA);
        #endif
        
        float angle = pupil_fit_cur.angle * PI / 180;
        float width = pupil_fit_cur.size.width*0.5;
        float height = pupil_fit_cur.size.height*0.5;
        cvLine(ipl_arr[I_CPRT_0],
               cvPoint(pupil_fit_cur.center.x - cos(angle) * width, pupil_fit_cur.center.y - sin(angle) * width),
               cvPoint(pupil_fit_cur.center.x + cos(angle) * width, pupil_fit_cur.center.y + sin(angle) * width),
               CV_RGB(0, 255, 255), 1, CV_AA);
        cvLine(ipl_arr[I_CPRT_0],
               cvPoint(pupil_fit_cur.center.x + sin(angle) * height, pupil_fit_cur.center.y - cos(angle) * height),
               cvPoint(pupil_fit_cur.center.x - sin(angle) * height, pupil_fit_cur.center.y + cos(angle) * height),
               CV_RGB(0, 0, 255), 1, CV_AA);
        
        cvEllipse(ipl_arr[I_CPRT_0],
                  cvPoint(pupil_fit_cur.center.x, pupil_fit_cur.center.y),
                  cvSize(width, height),
                  pupil_fit_cur.angle, 0, 360.0, CV_RGB(0, 0, 255), 1, CV_AA);
        cvCircle(ipl_arr[I_CPRT_0], cvPoint(pupil_fit_cur.center.x, pupil_fit_cur.center.y), 4, CV_RGB(0, 0, 255), 3, CV_AA);
    
        if (!isSettingRoi&&( is_circle_exist || is_center_dotted)){
            cvCircle(ipl_arr[I_CPRT_0], cvPoint(d_circle_cntx, d_circle_cnty), d_circle_radius, CV_RGB(0,0,255), 3, CV_AA);
        } else if(isSettingRoi &&(is_first_pt_dotted || is_rect_exist)){
            cvRectangle(ipl_arr[I_CPRT_0], cvPoint(d_rect_pt1x, d_rect_pt1y), cvPoint(d_rect_pt2x, d_rect_pt2y), CV_RGB(0,0,255), 3, CV_AA);
        }
        UNLOCK_THREAD_WITH_MUTEX(&dm_mutex);

        #if DOMINANT_MOVE
            if (captureCount > 0){
                moveImgAndSubstract(ipl_arr[I_GSRC], ipl_arr[I_GREF_0], ipl_arr[I_GPRT_1], ipl_arr[I_GPRT_0], -(int)translation.x, -(int)translation.y);
            }
        
            movement->getImages(ipl_arr[I_GPRT_0], 7);
            movement->getImages(ipl_arr[I_GPRT_2], 8);
            
            img_color_out0.setFromPixels( (uchar*)ipl_arr[I_CPRT_0]->imageData, cap_size.width, cap_size.height);
            img_debugging_out2.setFromPixels((uchar*)ipl_arr[I_GPRT_0]->imageData, cap_size.width, cap_size.height, OF_IMAGE_GRAYSCALE);
            img_debugging_out4.setFromPixels((uchar*)ipl_arr[I_GPRT_1]->imageData, cap_size.width, cap_size.height, OF_IMAGE_GRAYSCALE);
            img_debugging_out5.setFromPixels((uchar*)ipl_arr[I_GPRT_2]->imageData, cap_size.width, cap_size.height, OF_IMAGE_GRAYSCALE);

        #else

            detector.getImages(ipl_arr[I_GPRT_0], 0);
            detector.getImages(ipl_arr[I_GPRT_1], 1);
            detector.getImages(ipl_arr[I_GPRT_2], 2);
            detector.getImages(ipl_arr[I_GPRT_3], 3);
            detector.getImages(ipl_arr[I_GPRT_4], 4);
            detector.getImages(ipl_arr[I_GPRT_5], 5);

            detector.getColorImages(ipl_arr[I_CPRT_1], 0);
    
            detector.getMaskHistogram(hist_iris);
            detector.getMaskHistDerivatives(hist_deriv);
            if(pupil_found)
            detector.getPeakValleyList(peakList, valleyList);
//            if(pupil_found)
    
//            int detectorThreshold = detector.getAlgorithm() == ROBUST_AND_NEAR_PAST_FIXED_THRESHOLD ? detector.getManualThreshold() : detector.getThreshold();
            
            #if COMPARE_ALGORITHM
                int compareThreshold = compareDetector.getAlgorithm() == ROBUST_AND_NEAR_PAST_FIXED_THRESHOLD ? compareDetector.getManualThreshold() : compareDetector.getThreshold();
                setHistImg(hist_iris, ipl_arr[I_CHIST_0], detectorThreshold, compareThreshold);
            
            #else
//    if(pupil_found){
                if (setting_manual_threshold || manual_threshold_fixed){
                    setBottomHist(hist_iris, ipl_arr[I_CHIST_0], manualThrshold);
                    setBottomHist(hist_deriv, ipl_arr[I_CHIST_1], manualThrshold);
                } else {
                    setBottomHist(hist_iris, ipl_arr[I_CHIST_0], peakList, valleyList);
                    setBottomHist(hist_deriv, ipl_arr[I_CHIST_1], peakList, valleyList);
                }
//    }
            #endif
        
            img_color_out0.setFromPixels( (uchar*)ipl_arr[I_CPRT_0]->imageData, cap_size.width, cap_size.height);
            img_color_out1.setFromPixels((uchar*)ipl_arr[I_CHIST_1]->imageData, 256, cap_size.height);
            img_color_out2.setFromPixels((uchar*)ipl_arr[I_CHIST_0]->imageData, 256, cap_size.height);
            img_debugging_out1.setFromPixels((uchar*)ipl_arr[I_GPRT_0]->imageData, cap_size.width, cap_size.height, OF_IMAGE_GRAYSCALE);
            img_debugging_out2.setFromPixels((uchar*)ipl_arr[I_GPRT_1]->imageData, cap_size.width, cap_size.height, OF_IMAGE_GRAYSCALE);
            img_debugging_out3.setFromPixels((uchar*)ipl_arr[I_GPRT_2]->imageData, cap_size.width, cap_size.height, OF_IMAGE_GRAYSCALE);
            img_debugging_out4.setFromPixels((uchar*)ipl_arr[I_GPRT_3]->imageData, cap_size.width, cap_size.height, OF_IMAGE_GRAYSCALE);
            img_debugging_out5.setFromPixels((uchar*)ipl_arr[I_GPRT_4]->imageData, cap_size.width, cap_size.height, OF_IMAGE_GRAYSCALE);
            img_debugging_out6.setFromPixels((uchar*)ipl_arr[I_GPRT_5]->imageData, cap_size.width, cap_size.height, OF_IMAGE_GRAYSCALE);

        #endif
        
    #endif
    
}


void EyeTracker::moveImgAndSubstract(IplImage *src, IplImage *base, IplImage *dst, IplImage *tmp, int xdiff, int ydiff){
#if DOMINANT_MOVE
    uchar* src_ptr = (uchar*)src->imageData;
    uchar* tmp_ptr = (uchar*)tmp->imageData;

    cvSetZero(tmp);
    LOCK_THREAD_WITH_MUTEX(&dm_mutex);
    if (abs(xdiff) < CAM_RES_X && abs(ydiff) < CAM_RES_Y) {
        for (int r = MAX(0, -ydiff); r < src->height + MIN(0, -ydiff); r++) {
            uchar *src_ptr = (uchar *) src->imageData + (r + ydiff) * src->widthStep + xdiff;
            uchar *tmp_ptr = (uchar *) tmp->imageData + r * tmp->widthStep;
            for (int c = MAX(0, -xdiff); c < src->width + MIN(0, -xdiff); c++) {
                *tmp_ptr++ = *src_ptr++;
            }
        }
    }
    UNLOCK_THREAD_WITH_MUTEX(&dm_mutex);
    
    if (base != NULL){
        uchar* dst_ptr = (uchar*)dst->imageData;
        uchar* base_ptr = (uchar*)base->imageData;
        tmp_ptr = (uchar*)tmp->imageData;
        for (int r = 0 ; r < tmp->height ; r++){
            dst_ptr = (uchar*)dst->imageData + r * dst->widthStep;
            base_ptr = (uchar*)base->imageData + r * base->widthStep;
            tmp_ptr = (uchar*)tmp->imageData + r * tmp->widthStep;
            for (int c = 0 ; c < tmp->width ; c++){
                *dst_ptr++ = abs(*tmp_ptr++ - *base_ptr++);
            }
        }
    }
#endif
}


bool EyeTracker::getFrame( IplImage *ipl_csrc ){
	#if CAMERA_LIBUVC
        LOCK_THREAD_WITH_MUTEX(&dm_mutex);
        bool res = uvcCamera->getFrame();
        UNLOCK_THREAD_WITH_MUTEX(&dm_mutex);

        return res;

    #elif CAMERA_OFX
        if(cap_context.mode==CV_USE_LIVE_VIDEO){
            cap_context.vg.update();
            if( cap_context.vg.isFrameNew() ){
                memcpy(ipl_csrc->imageData, cap_context.vg.getPixels(), ipl_csrc->imageSize);
                return true;
            }
            return false;
        }
        
        if(cap_context.mode==CV_USE_VIDEO){
            bool willBenewFrame = false;
            if( bVideoStopped ){
                if (videoStep == 1){
                    videoStep = 0;
                    cap_context.vp.nextFrame();					
                    willBenewFrame = true;
                } else if (videoStep == -1){
                    videoStep = 0;
                    cap_context.vp.previousFrame();
                    willBenewFrame = true;
                }
            } else {
                cap_context.vp.update();
                if (cap_context.vp.isFrameNew()) willBenewFrame = true;
            }
            
            if( willBenewFrame ){
                memcpy(ipl_csrc->imageData, cap_context.vp.getPixels(), ipl_csrc->imageSize);
                return true;
            }
            return false;
        }
        return false;
    
    #endif

}


#if USE_CALIBRATION_PROCESS // for Andorid whole calibration parts

void EyeTracker::setRecentCalibrationInfo(float* srcPoint, float* dstPoint) {
    calibrationData->setRecentInfo(srcPoint, dstPoint);
}


void EyeTracker::initCalibrationProcess(){
    if(PRINT_DEBUG) ofLogNotice("EyeTracker") << "initCalibrationProcess";
    calibrationTime = 0;
    curCalibrationStep = 0;
    currentNumberOfGazePoint = 0;

    bCalibrationProcessDone=false;
}

bool EyeTracker::CalibrationProcess(){
        if(PRINT_SETFRAME) ofLogNotice("EyeTracker") << "calibrationProcess";
        bool calibrationDoNextStep = false;
        if(LIKELY(isInitialized) && cameraFrameNew) {
            cameraFrameNew = false;
            ofVec2f fixedResult;

            CvPoint2D32f filteredData;
            filteredData = guess_pt;
            calibrationFilter->Filter(filteredData.x, filteredData.y);
            if(PRINT_SETFRAME) ofLogNotice("EyeTracker") << "filtered Point : " << filteredData.x << ", " << filteredData.y;
            ofVec2f gazePoint(filteredData.x, filteredData.y);
            if (ofGetElapsedTimeMillis() - calibrationTime > CALIBRATION_ANIMATION_TIME) {
                if (guess_pt.x != 0 && !((currentNumberOfGazePoint != 0) &&
                                         gazeHistory[currentNumberOfGazePoint] == gazePoint)) {
                    currentNumberOfGazePoint;
                    if (currentNumberOfGazePoint < MAX_NUMBER_GAZEPOINT)
                        currentNumberOfGazePoint++;
                    else
                        for (int i = 1; i < MAX_NUMBER_GAZEPOINT; i++)
                            gazeHistory[i - 1] = gazeHistory[i];

                    gazeHistory[currentNumberOfGazePoint - 1].set(gazePoint);

                    if (currentNumberOfGazePoint >= MIN_NUMBER_LIMIT_GAZEPOINT) {
                        calibrationDoNextStep = isCalibrationPointFixed(fixedResult, gazeHistory, currentNumberOfGazePoint);
                    }
                }
            }
            if (calibrationDoNextStep) {
                calibrationTime = ofGetElapsedTimeMillis();

                float srcPoint[18]; //calibration data
                calibrationData->setSrcPoints(fixedResult, calibrationOrder[curCalibrationStep].y, calibrationOrder[curCalibrationStep].x, srcPoint);
                
                #if DOMINANT_MOVE
                    if (captureCount == 0) capture = true;
                
                #endif
                
                #if APP_TYPE == APP_TYPE_CUSTOM_ANDROID
                    if (++curCalibrationStep < NUMBER_CALIBRATION_POINTS) {
                        VCAndroidTool::getInstance().doNextStepCallback(calibrationOrder[curCalibrationStep].x, calibrationOrder[curCalibrationStep].y,srcPoint);
                    } else {
                        bCalibrationProcessDone = true;
                        VCAndroidTool::getInstance().doNextStepCallback(-1, -1, srcPoint);

                        #if DOMINANT_MOVE && THREAD_PTHREAD
                            start_dm_thread();
                        #endif
                    }
                #endif

            }
        }
        return bCalibrationProcessDone;
}


bool EyeTracker::isCalibrationPointFixed(ofVec2f &result,ofVec2f *history, int numberOfPoints){
    return isCalibrationPointFixedRecursive(result, history, numberOfPoints, numberOfPoints);
}

bool EyeTracker::isCalibrationPointFixedRecursive(ofVec2f &result,ofVec2f *history, int numberOfPoints, int numberOfCurrentPoints){
    ofVec2f center;
    float distance[numberOfCurrentPoints];
    center.average(history, numberOfCurrentPoints);

    for ( int i = 0; i < numberOfCurrentPoints ; i ++ ) {
        distance[i] = center.distanceSquared(history[i]);
    }

    float largestDistance = 0;
    int largestIndex = 0;
    for ( int i = 0 ; i < numberOfCurrentPoints ; i++ ) {
        if (distance[i] > largestDistance) {
            largestDistance = distance[i];
            largestIndex = i;
        }
    }
    if (largestDistance > circleSizeFailLimit) {
        result.set(0,0);
        currentNumberOfGazePoint = 0;
        return false;
    }
    if (largestDistance > circleSizeIncluding) {
        if (numberOfCurrentPoints - 1 <= numberOfPoints * pointsIncludingGazePointFixed) {
            result.set(0,0);
            return false;
        } else {
            ofVec2f *nextHistory = new ofVec2f[numberOfCurrentPoints-1];
            for ( int i = 0 ; i < numberOfCurrentPoints-1 ; i ++ ) {
                if (i<largestIndex) nextHistory[i] = history[i];
                else nextHistory[i] = history[i+1];
            }
            bool returnValue = isCalibrationPointFixedRecursive(result, nextHistory, numberOfPoints, numberOfCurrentPoints-1);
            delete [] nextHistory;
            nextHistory = NULL;
            return returnValue;
        }
    } else {
        ofLog() << "Calibration Do next Step !";
        result.set(center.x,center.y);
        currentNumberOfGazePoint = 0;
        return true;
    }
}
#endif // USE_CALIBRATION_PROCESS


#if (PRINT_IMAGES | PRINT_IMAGES_MINIMALIZE)
void EyeTracker::setHistImg(float* hist, IplImage* dest, int thres, int thresh2){
    cvSetZero(dest);
    int max_val = 0;
    int hist_print[256];
    uchar* img_hist = (uchar*)(dest->imageData);
    int step = dest->widthStep;
    
    for (int i = 0 ; i < 256 ; i ++ ){
        if ( abs(hist[i]) > max_val ) max_val = abs(hist[i]);
    }
    
    if (max_val == 0) return;
    for (int i = 0 ; i < 256 ; i ++ ){
        hist_print[i] = (max_val-(float)hist[i])/max_val*((float)dest->height/2);
    }
    
    
    for (int column = 0 ; column < 256 ; column ++){
        int lowVal = min((int)dest->height/2, (int)hist_print[column]);
        int highVal = max((int)dest->height/2, (int)hist_print[column]);
        for (int row = lowVal ; row < highVal ; row ++){
            img_hist[row*step + column * 3] = 255;
            img_hist[row*step + column * 3 + 1] = 255;
            img_hist[row*step + column * 3 + 2] = 255;
        }
    }
}

void EyeTracker::setHistImg(float *hist, IplImage *dest, vector<int> peak, vector<int> valley){
    cvSetZero(dest);
    float max_val = 0;
    float hist_print[256];
    uchar* img_hist = (uchar*)(dest->imageData);
    int step = dest->widthStep;
    
    
    for (int i = 0 ; i < 256 ; i ++ ){
        if ( abs(hist[i]) > max_val ) max_val = abs(hist[i]);
    }
    
    if (max_val == 0) return;
    for (int i = 0 ; i < 256 ; i ++ ){
        hist_print[i] = (max_val-(float)hist[i])/max_val*((float)dest->height/2);
    }
    
    
    for (int column = 0 ; column < 256 ; column ++){
        int lowVal = min((int)dest->height/2, (int)hist_print[column]);
        int highVal = max((int)dest->height/2, (int)hist_print[column]);
        for (int row = lowVal ; row < highVal ; row ++){
            img_hist[row*step + column * 3] = 255;
            img_hist[row*step + column * 3 + 1] = 255;
            img_hist[row*step + column * 3 + 2] = 255;
        }
    }
}

void EyeTracker::setBottomHist(float *hist, IplImage *dest, vector<int> peak, vector<int> valley){
    cvSetZero(dest);
    float max_val = 0;
    float hist_print[256];
    uchar* img_hist = (uchar*)(dest->imageData);
    int step = dest->widthStep;
    
    
    for (int i = 0 ; i < 256 ; i ++ ){
        if ( hist[i] > max_val ) max_val = hist[i];
    }
    
    if (max_val == 0) return;
    for (int i = 0 ; i < 256 ; i ++ ){
        hist_print[i] = (int)((float)hist[i]/max_val*dest->height);
    }
    
    
    for (int column = 0 ; column < 256 ; column ++){
        for (int row = 0 ; row < dest->height ; row ++){
            if (hist_print[column] >= dest->height - row) {
                for ( int i = row ; i < cap_size.height ; i ++ ) {
                    img_hist[i*step + column * 3] = 255;
                    img_hist[i*step + column * 3 + 1] = 255;
                    img_hist[i*step + column * 3 + 2] = 255;
                }
                break;
            }
        }
    }
    
    for (int curPeak : peak){
        for (int row = 0 ; row < dest->height ; row ++){
            img_hist[row*step + curPeak * 3] = 255;
            img_hist[row*step + curPeak * 3 + 1] = 0;
            img_hist[row*step + curPeak * 3 + 2] = 0;
        }
    }

    for (int curValley : valley){
        for (int row = 0 ; row < dest->height ; row ++){
            img_hist[row*step + curValley * 3] = 255;
            img_hist[row*step + curValley * 3 + 1] = 255;
            img_hist[row*step + curValley * 3 + 2] = 0;
        }
    }
}

void EyeTracker::setBottomHist(float *hist, IplImage *dest, int thres, int thresh2){
    cvSetZero(dest);
    float max_val = 0;
    float hist_print[256];
    uchar* img_hist = (uchar*)(dest->imageData);
    int step = dest->widthStep;
    
    
    for (int i = 0 ; i < 256 ; i ++ ){
        if ( hist[i] > max_val ) max_val = hist[i];
    }
    
    if (max_val == 0) return;
    for (int i = 0 ; i < 256 ; i ++ ){
        hist_print[i] = (int)((float)hist[i]/max_val*dest->height);
    }
    
    
    for (int column = 0 ; column < 256 ; column ++){
        for (int row = 0 ; row < dest->height ; row ++){
            if (hist_print[column] >= dest->height - row) {
                for ( int i = row ; i < cap_size.height ; i ++ ) {
                    img_hist[i*step + column * 3] = 255;
                    img_hist[i*step + column * 3 + 1] = 255;
                    img_hist[i*step + column * 3 + 2] = 255;
                }
                break;
            }
        }
    }
    
    for (int column = 0 ; column < 256 ; column ++){
        for (int row = 0 ; row < dest->height ; row ++){
            if (hist_print[column] >= dest->height - row) {
                for ( int i = row ; i < cap_size.height ; i ++ ) {
                    img_hist[i*step + column * 3] = 255;
                    img_hist[i*step + column * 3 + 1] = 255;
                    img_hist[i*step + column * 3 + 2] = 255;
                }
                break;
            }
        }
    }

    for (int row = 0 ; row < dest->height ; row ++){
        img_hist[row*step + thres * 3] = 255;
        img_hist[row*step + thres * 3 + 1] = 0;
        img_hist[row*step + thres * 3 + 2] = 0;
        if (thresh2 != 0) {
            img_hist[row*step + thresh2 * 3] = 0;
            img_hist[row*step + thresh2 * 3 + 1] = 255;
            img_hist[row*step + thresh2 * 3 + 2] = 255;
        }
    }
}


void EyeTracker::printMask(){
    int count = 0;
    string printStr = "";
    
    int *startCol = (int*)malloc(sizeof(int) * cap_size.height);
    int *endCol = (int*)malloc(sizeof(int) * cap_size.height);
    
    double r_square = d_circle_radius * d_circle_radius;
    for (int row = 0 ; row < cap_size.height ; row ++){
        double y_abs = (d_circle_cnty - row) * (d_circle_cnty - row);
        double x_abs;
        startCol[row] = cap_size.width;
        for (int column = 0 ; column < cap_size.width ; column++){
            x_abs = (d_circle_cntx - column)*(d_circle_cntx - column);
            if (x_abs + y_abs < r_square){
                startCol[row] = column;
                break;
            }
        }
    }
    
    for (int row = 0 ; row < cap_size.height ; row ++){
        if (startCol[row] == cap_size.width){
            endCol[row] = -1;
            continue;
        }
        double y_abs = (d_circle_cnty - row) * (d_circle_cnty - row);
        double x_abs;
        
        endCol[row] = cap_size.width-1;
        for (int column = startCol[row] + 1 ; column < cap_size.width ; column ++){
            x_abs = (d_circle_cntx - column)*(d_circle_cntx - column);
            if (x_abs + y_abs > r_square){
                endCol[row] = column;
                break;
            }
        }
    }
    
    
    for (int row = 0 ; row < cap_size.height ; row ++){
        printStr += ofToString(startCol[row]) +", ";
        if (++count == 10){
            std::cout << printStr << endl;
            printStr = "";
            count = 0;
        }
    }
    printf("\n\n");
    
    for (int row = 0 ; row < cap_size.height ; row ++){
        printStr += ofToString(endCol[row]) +", ";
        if (++count == 9){
            std::cout << printStr << endl;
            printStr = "";
            count = 0;
        }
    }
}

/**
 *	This method is OpenFrameworks' draw tick function.
 *	@date 2012/05/07
 */
void EyeTracker::draw(){
    if(PRINT_SETFRAME) ofLogNotice("EyeTracker") << "draw";
    
    ofEnableBlendMode(OF_BLENDMODE_ALPHA);
    ofBackground(64,64,64,255);
    
    drawStatusCheck();

    #if VIDEO_RECORDING
    if(bRecCurDisplay){
        int iSize=30;
        ofSetColor(255,0,0,200);
        ofRect(ofGetWindowWidth()-iSize*6/5, ofGetWindowHeight()-iSize*6/5, iSize, iSize);
        
        img_window_capture.setFromPixels((uchar*)ipl_arr[I_CPRT_0]->imageData, cap_size.width, cap_size.height, OF_IMAGE_COLOR);
        vs->add_frame(img_window_capture);
    }
    
    if(bRecCurDisplayforTest){
        int iSize=30;
        ofSetColor(255,255,0,200);
        ofRect(ofGetWindowWidth()-iSize*6/5, ofGetWindowHeight()-iSize*6/5, iSize, iSize);
        
        img_window_capture.grabScreen(0, 0, ofGetWindowWidth(), ofGetWindowHeight());
        vs->add_frame(img_window_capture);
    }
    #endif
    gui.setPosition(ofGetWindowWidth() - gui.getWidth(), ofGetWindowHeight() - gui.getHeight());
    gui.draw();
    
//    string str = "framerate is ";
//    str += ofToString(ofGetFrameRate(), 2)+"fps";
//    ofSetWindowTitle(str);
}

//madmoon
void EyeTracker::drawStatusCheck(){
    ofSetColor(255,255,255,255);
    #if PRINT_IMAGES_MINIMALIZE

        int space_x = ofGetWindowWidth()/9;
        int space_y = ofGetWindowWidth()/9;

        int img_size_width = ofGetWindowWidth() / 3;
        int img_size_height = ofGetWindowWidth() / 3 * cap_size.height / cap_size.width;

        img_debugging_out1.draw(space_x, space_y, img_size_width-1,img_size_height); //View Cam gray image
        img_debugging_out2.draw(space_x + img_size_width + ofGetWindowWidth()/10, space_y,img_size_width-1,img_size_height); //Eye tracking image

    #elif PRINT_IMAGES

        int space_x = 0;//ofGetWindowWidth()/6;
        int space_y = 0;

        #if DOMINANT_MOVE
            int img_size_width = ofGetWindowWidth() / 4;
            int img_size_height = ofGetWindowWidth() / 4 * cap_size.height / cap_size.width;

            img_color_out0.draw(space_x, space_y,img_size_width-1,img_size_height); //Eye tracking image
            img_debugging_out2.draw(space_x + img_size_width * 1, space_y, img_size_width-1,img_size_height);
            img_debugging_out4.draw(space_x + img_size_width * 2, space_y, img_size_width-1,img_size_height);
            img_debugging_out5.draw(space_x + img_size_width * 3, space_y, img_size_width-1,img_size_height);
        #else
            int img_size_width = ofGetWindowWidth() / 5;
            int img_size_height = ofGetWindowWidth() / 5 * cap_size.height / cap_size.width;
            

            img_color_out2.draw(    space_x + img_size_width*0, space_y,                    img_size_width-1,img_size_height);
            img_color_out0.draw(    space_x + img_size_width*1, space_y,                    img_size_width-1,img_size_height); //Eye tracking image
            img_debugging_out1.draw(space_x + img_size_width*2, space_y,                    img_size_width-1,img_size_height);
            img_debugging_out2.draw(space_x + img_size_width*3, space_y,                    img_size_width-1,img_size_height);
            img_debugging_out3.draw(space_x + img_size_width*4, space_y,                    img_size_width-1,img_size_height);
            img_color_out1.draw(    space_x + img_size_width*0, space_y + img_size_height,  img_size_width-1,img_size_height);
            img_debugging_out4.draw(space_x + img_size_width*1, space_y + img_size_height,  img_size_width-1,img_size_height);
            img_debugging_out5.draw(space_x + img_size_width*2, space_y + img_size_height,  img_size_width-1,img_size_height);
            img_debugging_out6.draw(space_x + img_size_width*3, space_y + img_size_height,  img_size_width-1,img_size_height);

        #endif
    
    #endif
    
//    drawLetterBoard();
}

void EyeTracker::drawLetterBoard()
{
    ofSetColor(255,255,255,255);
    
    float sy = bigFont0.getLineHeight();

    sprintf(reportStr, "current FPS : %.1f", fps);
    bigFont0.drawString(reportStr,ofGetWindowWidth()/7,ofGetWindowHeight()/2 + 200 - sy);
    sprintf(reportStr, "proccesing Ratio : %.2f", processingRatio);
    bigFont0.drawString(reportStr,ofGetWindowWidth()/7,ofGetWindowHeight()/2 + 200 + sy);

}
#endif // IMAGE OUTPUT


#if (VIDEO_OFX | CAMERA_OFX)
bool EyeTracker::initVideo( video_context *vid_context,CvSize *size, int iCamNum){
    if(vid_context->mode==CV_USE_VIDEO){
        vid_context->vp.setSpeed(2.0);
        vid_context->vp.setUseTexture(true);
        vid_context->vp.loadMovie("video/eye.mov");
        vid_context->vp.play();
		float duration = vid_context->vp.getDuration();
		float point_time[36];
		*point_time = AlgorithmEvaluation::SetPointTime(duration);

        if (vid_context->vp.isLoaded()) {
            *size = cvSize( vid_context->vp.getWidth(), vid_context->vp.getHeight() );
        }
        vid_context->ready = vid_context->vp.isLoaded();
        return vid_context->vp.isLoaded();
    }

    if(vid_context->mode==CV_USE_LIVE_VIDEO){
        vid_context->vg.setVerbose(true);
        vid_context->vg.setDeviceID(iCamNum);
        vid_context->vg.setDesiredFrameRate(40);
        vid_context->vg.setUseTexture(true);
        vid_context->vg.videoSettings();
        vid_context->vg.initGrabber( size->width, size->height );
        vid_context->ready = true;
        *size = cvSize( vid_context->vg.getWidth(), vid_context->vg.getHeight() );
        return true;
    }

    return false;
}
#endif

#if APP_TYPE == APP_TYPE_OFX_NORMAL & VIDEO_OFX
void EyeTracker::settingToggleCallback(bool &setting){
    if (settingThreshold){
        bVideoStopped = true;
        cap_context.vp.setSpeed(1.0f);
        cap_context.vp.setPaused(bVideoStopped);
        
        wait_set_threshold = true;
        setting_manual_threshold = false;
        manual_threshold_fixed = false;
    }
}
#endif

#if USE_KEYBOARD_INPUT
void EyeTracker::keyPressed(int key){

    switch(key){

#if APP_TYPE == APP_TYPE_OFX_NORMAL
        case '=':
            if (is_circle_exist) d_circle_radius+=2;
            break;
        case '-':
            if (is_circle_exist) d_circle_radius-=2;
            break;
        case OF_KEY_LEFT:
            if (isSettingRoi) {
                d_rect_pt1x -= 2;
                d_rect_pt2x -= 2;
            } else
                d_circle_cntx-=2;
            break;
        case OF_KEY_UP:
            if (isSettingRoi) {
                d_rect_pt1y -= 2;
                d_rect_pt2y -= 2;
            } else
                d_circle_cnty-=2;
            break;
        case OF_KEY_RIGHT:
            if (isSettingRoi) {
                d_rect_pt1x += 2;
                d_rect_pt2x += 2;
            } else
                d_circle_cntx+=2;
            break;
        case OF_KEY_DOWN:
            if (isSettingRoi) {
                d_rect_pt1y += 2;
                d_rect_pt2y += 2;
            } else
                d_circle_cnty+=2;
            break;
            
        case OF_KEY_RETURN:
            printMask();
            break;
#endif

        case 'R':
            bRecCurDisplayforTest =! bRecCurDisplayforTest;
            if (bRecCurDisplayforTest) {
                bStartRec = true;
                ofSetFrameRate(FPS_LO);
                vs->start_rec(ofGetWindowWidth(), ofGetWindowHeight(), FPS_LO);
            }
            else
            {
                if(bStartRec)
                {
                    ofSetFrameRate(FPS_HI);
                    vs->finish_rec();
                    bStartRec=false;
                }
            }
            break;
        case 'r':
            bRecCurDisplay=!bRecCurDisplay;
            if(bRecCurDisplay)
            {
                bStartRec=true;
                ofSetFrameRate(FPS_LO);
                vs->start_rec(cap_size.width, cap_size.height, FPS_LO);
            }
            else
            {
                if(bStartRec)
                {
                    ofSetFrameRate(FPS_HI);
                    vs->finish_rec();
                    bStartRec=false;
                }
            }
            break;

#if CAMERA_LIBUVC
        case 'f':
            uvcCamera->raiseExposure(10);
            break;
            
        case 'd':
            uvcCamera->reduceExposure(10);
            break;
            
        case 's':
            uvcCamera->raiseIris(10);
            break;
            
        case 'a':
            uvcCamera->reduceIris(10);
            break;
            
        case 'v':
            uvcCamera->raiseBright(10);
            break;
            
        case 'c':
            uvcCamera->reduceBright(10);
            break;

        case 'x':
            uvcCamera->raiseContrast(10);
            break;
            
        case 'z':
            uvcCamera->reduceContrast(10);
            break;
#endif
            
#if VIDEO_OFX
        case 's':
            playSpeed = playSpeed - 0.1;
            cap_context.vp.setSpeed(playSpeed);
            break;


        case 'f':
            playSpeed = playSpeed + 0.1;
            cap_context.vp.setSpeed(playSpeed);
            break;

            // toggle drawing perkinje points
        case 'd':
            playSpeed = 1.0;
            cap_context.vp.setSpeed(playSpeed);
            break;

        case 'n':
            if(bVideoStopped){
                videoStep = 1;
            }
            break;

        case 'm':
            if(bVideoStopped){
                videoStep = -1;
            }
            break;

        case 'p':
            bVideoStopped = !bVideoStopped;
            cap_context.vp.setSpeed(1.0f);
            cap_context.vp.setPaused(bVideoStopped);
//           (ca if (bVideoStopped) capture = true;
            break;
#endif
            
#if DOMINANT_MOVE
        case 'o':
            capture = true;
            break;
            
#endif
            
#if COMPARE_ALGORITHM
		case '+':
			if(compareAlgorithm == ROBUST_AND_NEAR_PAST_FIXED_THRESHOLD)
				compareDetector.setManualThreshold(compareDetector.getManualThreshold() + 1);
			if(detector.getAlgorithm() == ROBUST_AND_NEAR_PAST_FIXED_THRESHOLD)
				detector.setManualThreshold(compareDetector.getManualThreshold() + 1);
			break;
            
		case '-':
			if (compareAlgorithm == ROBUST_AND_NEAR_PAST_FIXED_THRESHOLD)
				compareDetector.setManualThreshold(compareDetector.getManualThreshold() - 1);
			if (detector.getAlgorithm() == ROBUST_AND_NEAR_PAST_FIXED_THRESHOLD)
				detector.setManualThreshold(compareDetector.getManualThreshold() - 1);
			break;
            
		case '0':
			//if (compareAlgorithm == ROBUST_AND_NEAR_PAST_FIXED_THRESHOLD)
			if(compareAlgorithm != NONE)
				compareDetector.findManualThreshold();
			//if (detector.getAlgorithm() == ROBUST_AND_NEAR_PAST_FIXED_THRESHOLD)
				detector.findManualThreshold();
			break;


		// CHANGE : '.'�� ������ ������ ���� ����. ','�� ������ ������ ���� �Ϸ� �� ��� ���(������ ������Ŵ)
		case '.': //start data collecting
			
			detector.startCollecting();
			if (compareAlgorithm != NONE)
				compareDetector.startCollecting();

			break;

		case ',': // stop data collecting

			detector.stopCollecting();
			if (compareAlgorithm != NONE)
				compareDetector.stopCollecting();

			if (!bVideoStopped) {
				bVideoStopped = !bVideoStopped;
				cap_context.vp.setSpeed(1.0f);
				cap_context.vp.setPaused(bVideoStopped);
			}

			break;
#endif
	
    }
}



//--------------------------------------------------------------
void EyeTracker::keyReleased(int key){

}


//--------------------------------------------------------------
void EyeTracker::mouseMoved(int x, int y ){
    string str = "mouse : (";
    str += ofToString(x) + ", ";
    str += ofToString(y) + ")";
    ofSetWindowTitle(str);
}

//--------------------------------------------------------------
void EyeTracker::mouseDragged(int x, int y, int button){
    if (setting_manual_threshold){
        if (button == 0){
            int screen_img_width = ofGetWindowWidth() / 5;
            int screen_img_height = ofGetWindowWidth() / 5 * cap_size.height / cap_size.width;
            
            manualThrshold = MIN(MAX((x - screen_img_width * 0) / (float)screen_img_width * 255, 0), 255);
            need_refresh = true;
        }
        return;
    }
    if (!isSettingRoi){
        if (button==0 && is_center_dotted){
            int screen_img_width = ofGetWindowWidth() / 5;
            int screen_img_height = ofGetWindowWidth() / 5 * cap_size.height / cap_size.width;
            
            
            int cur_x = ((float)x / screen_img_width) * cap_size.width;
            int cur_y = ((float)y / screen_img_height) * cap_size.height;
            
            d_circle_radius = sqrt((cur_x - d_circle_cntx) * (cur_x - d_circle_cntx) + (cur_y - d_circle_cnty) * (cur_y - d_circle_cnty));
//            ofLogNotice("EyeTracker") << "mouseDraged : " << x << ", " << y << " / " << d_circle_cntx << ", " << d_circle_cnty;
        }
    } else {
        if (button==0 && is_first_pt_dotted){
            int screen_img_width = ofGetWindowWidth() / 5;
            int screen_img_height = ofGetWindowWidth() / 5 * cap_size.height / cap_size.width;
            
            
            d_rect_pt2x = ((float)x / screen_img_width) * cap_size.width;
            d_rect_pt2y = ((float)y / screen_img_height) * cap_size.height;
        }
    }
}

//--------------------------------------------------------------
void EyeTracker::mousePressed(int x, int y, int button){
    if (wait_set_threshold) {
        if (button == 0){
            int screen_img_width = ofGetWindowWidth() / 5;
            int screen_img_height = ofGetWindowWidth() / 5 * cap_size.height / cap_size.width;

            if (x >= screen_img_width * 0 && x < screen_img_width * 1 && y > screen_img_height*0 && y < screen_img_height * 2){
                manualThrshold = (x - screen_img_width * 0) / (float)screen_img_width * 255;
                setting_manual_threshold = true;
                wait_set_threshold = false;
                need_refresh = true;
            }
        }
        return;
    }
    if (!isSettingRoi){
        if (button == 0 && !is_center_dotted) {
            int screen_img_width = ofGetWindowWidth() / 5;
            int screen_img_height = ofGetWindowWidth() / 5 * cap_size.height / cap_size.width;
            
            d_circle_cntx = ((float)x / screen_img_width) * cap_size.width;
            d_circle_cnty = ((float)y / screen_img_height) * cap_size.height;
            
            is_center_dotted = true;
            is_circle_exist = false;

        } else if (button == 2){
            is_center_dotted = false;
            is_circle_exist = false;
        }
    } else {
        if (button == 0 && !is_first_pt_dotted){
            int screen_img_width = ofGetWindowWidth() / 5;
            int screen_img_height = ofGetWindowWidth() / 5 * cap_size.height / cap_size.width;
            
            d_rect_pt1x = ((float)x / screen_img_width) * cap_size.width;
            d_rect_pt1y = ((float)y / screen_img_height) * cap_size.height;
            
            is_first_pt_dotted = true;
            is_rect_exist = false;

        }
    }
}

//--------------------------------------------------------------
void EyeTracker::mouseReleased(int x, int y, int button){
    if (setting_manual_threshold){
        if (button == 0){
            int screen_img_width = ofGetWindowWidth() / 5;
            int screen_img_height = ofGetWindowWidth() / 5 * cap_size.height / cap_size.width;
            
            manualThrshold = MIN(MAX((x - screen_img_width * 0) / (float)screen_img_width * 255,0), 255);
            setting_manual_threshold = false;
            manual_threshold_fixed = true;
            settingThreshold = false;
            need_refresh = true;
        }
        return;
    }
    if (!isSettingRoi){
        if (button == 0 && is_center_dotted){
            int screen_img_width = ofGetWindowWidth() / 5;
            int screen_img_height = ofGetWindowWidth() / 5 * cap_size.height / cap_size.width;
            
            
            int cur_x = ((float)x / screen_img_width) * cap_size.width;
            int cur_y = ((float)y / screen_img_height) * cap_size.height;
            
            d_circle_radius = sqrt((cur_x - d_circle_cntx) * (cur_x - d_circle_cntx) + (cur_y - d_circle_cnty) * (cur_y - d_circle_cnty));

            is_center_dotted = false;
            is_circle_exist = true;
        }
    } else {
        if (button==0 && is_first_pt_dotted){
            int screen_img_width = ofGetWindowWidth() / 5;
            int screen_img_height = ofGetWindowWidth() / 5 * cap_size.height / cap_size.width;
            
            
            d_rect_pt2x = ((float)x / screen_img_width) * cap_size.width;
            d_rect_pt2y = ((float)y / screen_img_height) * cap_size.height;
            is_first_pt_dotted = false;
            is_rect_exist = true;
        }
    }
}
#endif

#if defined(__DESKTOP_APP__)
//--------------------------------------------------------------
void EyeTracker::windowResized(int w, int h){

}

//--------------------------------------------------------------
void EyeTracker::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void EyeTracker::dragEvent(ofDragInfo dragInfo){

}
#endif

int touchCount = 0;
void EyeTracker::touchDown(int x, int y, int id){
#if DOMINANT_MOVE
    if (touchCount == 0) capture = true;
    else {
        printingDebug = !printingDebug;
    }
    touchCount++;
#endif
}



void EyeTracker::terminate(){
    #if defined(__VREX__) && defined(__ANDROID__)
        if(PRINT_DEBUG) ofLogNotice("EyeTracker") << "terminate";
        isInitialized = false;
        
        VCAndroidTool::getInstance().stopGazeCallback();
    #endif

    #if defined(__ANDROID__)
        pthread_mutex_lock(&camera_mutex);
        cameraFrameNew = false;
        pthread_cond_signal(&camera_cond);
        pthread_mutex_unlock(&camera_mutex);

        pthread_mutex_lock(&camera_mutex);
        pthread_mutex_unlock(&camera_mutex);

        pthread_mutex_destroy(&camera_mutex);
        pthread_mutex_destroy(&process_mutex);

        pthread_cond_destroy(&camera_cond);
        pthread_cond_destroy(&calibration_cond);

        delete calibrationData;
        delete _gfilter;
        _gfilter = NULL;

    #endif
}


