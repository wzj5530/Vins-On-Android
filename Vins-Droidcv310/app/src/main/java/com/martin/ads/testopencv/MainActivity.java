package com.martin.ads.testopencv;

import android.view.GestureDetector;
import android.view.GestureDetector.SimpleOnGestureListener;
import android.view.MotionEvent;

import android.app.Activity;
import android.hardware.Camera;
import android.os.Bundle;
import android.util.Log;
import android.view.ScaleGestureDetector;
import android.view.View;
import android.view.WindowManager;

import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.JavaCameraView;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;

public class MainActivity extends Activity implements
        CameraBridgeViewBase.CvCameraViewListener2,
        GestureDetector.OnGestureListener,
        ScaleGestureDetector.OnScaleGestureListener,
        View.OnTouchListener {

    private static final String    TAG = "MainActivity";
    private static final int IMG_Width = 640;
    private static final int IMG_Height = 360;
    //private static final int IMG_Width = 640;
    //private static final int IMG_Height = 480;

    private Mat                    mRgba;
    private Mat                    mIntermediateMat;
    private Mat                    mGray;
    private CameraBridgeViewBase   mOpenCvCameraView;

    private View sufure_view;

    float currentScale = 1;

    GestureDetector detector;

    private ScaleGestureDetector scaleGestureDetector;

    long currentTime = System.currentTimeMillis();

    static {
        System.loadLibrary("opencv_java3");
        System.loadLibrary("native-lib");
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.tutorial2_activity_surface_view);
        mOpenCvCameraView.setVisibility(CameraBridgeViewBase.VISIBLE);
        mOpenCvCameraView.setCvCameraViewListener(this);
        mOpenCvCameraView.setMaxFrameSize(IMG_Width,IMG_Height);
        mOpenCvCameraView.setClickable(true);
        //mOpenCvCameraView.setOnClickListener(new View.OnClickListener() {
        //    @Override
        //    public void onClick(View v) {
        //        Camera camera=((JavaCameraView)mOpenCvCameraView).getCamera();
        //        if (camera!=null) camera.autoFocus(null);
        //    }
        //});

        detector = new GestureDetector(this, this);

        sufure_view = findViewById(R.id.tutorial_view);
        sufure_view.setOnTouchListener(this);
        sufure_view.setLongClickable(true);

        scaleGestureDetector=new ScaleGestureDetector(this,this);
    }

    @Override
    public boolean onDown(MotionEvent e) {
        Log.i(getClass().getName(), "MotionEvent onDown");
        return false;
    }

    @Override
    public void onShowPress(MotionEvent e) {
        Log.i(getClass().getName(), "MotionEvent onShowPress");

    }

    @Override
    public boolean onSingleTapUp(MotionEvent e) {
        Log.i(getClass().getName(), "MotionEvent onSingleTapUp");
        return false;
    }

    @Override
    public void onLongPress(MotionEvent e) {
        Log.i(getClass().getName(), "MotionEvent onLongPress");
    }

    @Override
    public boolean onScroll(MotionEvent e1, MotionEvent e2, float distanceX, float distanceY) {
        Log.i(getClass().getName(), "MotionEvent onScroll");
        panGestureRecognizer(-distanceX * 10, -distanceY * 10);
        return false;
    }

    @Override
    public boolean onFling(MotionEvent e1, MotionEvent e2, float velocityX, float velocityY) {
        Log.i(getClass().getName(), "MotionEvent onFling");
        return false;
    }

    @Override
    public void onPause()
    {
        super.onPause();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();

        nativeVinsStopSensor();
    }

    @Override
    public void onResume()
    {
        super.onResume();
        if (!OpenCVLoader.initDebug()) {
            Log.d(TAG, "Internal OpenCV library not found. Using OpenCV Manager for initialization");
        } else {
            Log.d(TAG, "OpenCV library found inside package. Using it!");
            mOpenCvCameraView.enableView();
        }

        nativeVinsStartSensor();
    }

    public void onDestroy() {
        super.onDestroy();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();

        nativeVinsQuit();
    }

    public void onCameraViewStarted(int width, int height) {
        Log.d(TAG, "onCameraViewStarted w:"+width+"  h:"+height);
        mRgba = new Mat(height, width, CvType.CV_8UC4);
        mIntermediateMat = new Mat(height, width, CvType.CV_8UC4);
        mGray = new Mat(height, width, CvType.CV_8UC1);

        nativeVinsInitialise();
        nativeVinsStartSensor();
    }

    public void onCameraViewStopped() {
        mRgba.release();
        mGray.release();
        mIntermediateMat.release();

        nativeVinsStopSensor();
    }

    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        mRgba = inputFrame.rgba();
        mGray = inputFrame.gray();
        nativeProcessFrame(mGray.getNativeObjAddr(), mRgba.getNativeObjAddr());
        return mRgba;
    }
    @Override
    public boolean onTouch(View v, MotionEvent event) {
        int pointerCount = event.getPointerCount();
        if(pointerCount>1)
            return onTouchEvent(event);
        else
            return detector.onTouchEvent(event);
    }
    @Override
    public boolean onTouchEvent(MotionEvent event){
        return scaleGestureDetector.onTouchEvent(event);
    }
    @Override
    public boolean onScale(ScaleGestureDetector detector) {
        Log.d(TAG,"ScaleGestureDetector onScale");
        float scalefactor = detector.getScaleFactor();
        scaleGestureRecognizer(scalefactor);
        return true;
    }

    @Override
    public boolean onScaleBegin(ScaleGestureDetector detector) {
        //Log.d(TAG,"tctoulog onScaleBegin getFocusX " + detector.getFocusX()
        //        + " getCurrentSpanX "+ detector.getCurrentSpanX()
        //        +" getScaleFactor "+detector.getScaleFactor());

        return true;
    }

    @Override
    public void onScaleEnd(ScaleGestureDetector detector) {

        Log.d(TAG,"ScaleGestureDetector onScaleEnd");
    }

    public native String stringFromJNI();
    public native void nativeProcessFrame(long matAddrGr, long matAddrRgba);
    public native boolean nativeVinsInitialise();
    public native boolean nativeVinsStartSensor();
    public native boolean nativeVinsStopSensor();
    public native boolean nativeVinsQuit();

    public native void panGestureRecognizer(double vx_smooth, double vy_smooth);
    public native void scaleGestureRecognizer(double scale);

}