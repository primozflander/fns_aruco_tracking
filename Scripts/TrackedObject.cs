using System.Collections.Generic;
using UnityEngine;
using OpenCVForUnity.CoreModule;
using OpenCVForUnity.ArucoModule;
using OpenCVForUnity.UnityUtils;

public class TrackedObject
{
    public bool enableLowPassFilter = false;
    public bool refineMarkerDetection = true;
    public bool shouldMoveARCamera = false;
    float positionLowPass = 0.005f;
    float rotationLowPass = 2f;
    Board board;
    GameObject arGameObject;
    PoseData oldPoseData;
    Matrix4x4 ARM;
    DetectorParameters detectparams;


    public TrackedObject(GameObject arGameObject, List<Mat> objPoints, Dictionary dictionary, Mat ids, DetectorParameters detectparams)
    {
        this.detectparams = detectparams;
        this.arGameObject = arGameObject;
        this.board = Board.create(objPoints, dictionary, ids);
    }


    public void EstimatePose(Mat imageMat, Mat camMat, Camera cam, Mat recoveredIdxs, List<Mat> corners, List<Mat> rejectedCorners, Mat detectedIds)
    {
        if (refineMarkerDetection)
        {
            Aruco.refineDetectedMarkers(imageMat, board, corners, detectedIds, rejectedCorners, camMat, new Mat(), 10f, 3f, true, recoveredIdxs, detectparams);
        }
        Mat rvec = new Mat();
        Mat tvec = new Mat();
        int valid = Aruco.estimatePoseBoard(corners, detectedIds, board, camMat, new Mat(), rvec, tvec);
        if (valid > 0)
        {
            arGameObject.SetActive(true);
            UpdateARObjectTransform(rvec, tvec, cam);
        }
    }


    private void UpdateARObjectTransform(Mat rvec, Mat tvec, Camera cam)
    {
        double[] rvecArr = new double[3];
        rvec.get(0, 0, rvecArr);
        double[] tvecArr = new double[3];
        tvec.get(0, 0, tvecArr);
        PoseData poseData = ARUtils.ConvertRvecTvecToPoseData(rvecArr, tvecArr);
        if (enableLowPassFilter)
        {
            ARUtils.LowpassPoseData(ref oldPoseData, ref poseData, positionLowPass, rotationLowPass);
        }
        oldPoseData = poseData;
        ARM = ARUtils.ConvertPoseDataToMatrix(ref poseData, true);
        if (shouldMoveARCamera)
        {
            ARM = arGameObject.transform.localToWorldMatrix * ARM.inverse;
            ARUtils.SetTransformFromMatrix(cam.transform, ref ARM);
        }
        else
        {
            ARM = cam.transform.localToWorldMatrix * ARM;
            ARUtils.SetTransformFromMatrix(arGameObject.transform, ref ARM);
        }
    }
}
