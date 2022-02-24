using UnityEngine;
using UnityEngine.UI;
using UnityEngine.SceneManagement;
using System.Xml.Serialization;
using System.IO;
using System.Collections;
using System.Collections.Generic;
using OpenCVForUnity.CoreModule;
using OpenCVForUnity.ArucoModule;
using OpenCVForUnity.Calib3dModule;
using OpenCVForUnity.ImgprocModule;
using OpenCVForUnity.UnityUtils;
using OpenCVForUnity.UnityUtils.Helper;

namespace OpenCVForUnityExample
{
    [RequireComponent(typeof(WebCamTextureToMatHelper))]
    public class TrackedObjectTest : MonoBehaviour
    {
        public bool useStoredCameraParameters = false;
        public bool showRejectedCorners = false;
        public bool showDetectedMarkers = false;
        public bool applyEstimationPose = true;
        public bool refineMarkerDetection = true;
        public bool shouldMoveARCamera = false;
        // public bool enableLowPassFilter = false;
        public GameObject arGameObject1;
        public GameObject arGameObject2;
        public Camera arCamera;
        float markerLength = 0.1f;

        public RawImage background;
        // float positionLowPass = 0.005f;
        // float rotationLowPass = 2f;
        // PoseData oldPoseObj1Data;
        // PoseData oldPoseObj2Data;
        Texture2D texture;
  
        WebCamTextureToMatHelper webCamTextureToMatHelper;
        Mat rgbMat;
        Mat camMatrix;
        MatOfDouble distCoeffs;
        Matrix4x4 ARM;
        Mat ids;
        List<Mat> corners;
        List<Mat> rejectedCorners;
        Mat rvecs;
        Mat tvecs;
        Mat rotMat;
        DetectorParameters detectorParams;
        Dictionary dictionary;
        FpsMonitor fpsMonitor;
        Mat recoveredIdxs;
        // custom boards
        Board object1Board;
        Board object2Board;
        Mat object1Ids;
        Mat object2Ids;
        TrackedObject object1;
        TrackedObject object2;

        void Start()
        {
            fpsMonitor = GetComponent<FpsMonitor>();
            webCamTextureToMatHelper = gameObject.GetComponent<WebCamTextureToMatHelper>();
            webCamTextureToMatHelper.Initialize();
        }

        /// <summary>
        /// Raises the webcam texture to mat helper initialized event.
        /// </summary>
        public void OnWebCamTextureToMatHelperInitialized()
        {
            Debug.Log("OnWebCamTextureToMatHelperInitialized");

            Mat webCamTextureMat = webCamTextureToMatHelper.GetMat();

            texture = new Texture2D(webCamTextureMat.cols(), webCamTextureMat.rows(), TextureFormat.RGBA32, false);
            Utils.fastMatToTexture2D(webCamTextureMat, texture);

            // gameObject.GetComponent<Renderer>().material.mainTexture = texture;
            // background.texture = texture;
            gameObject.transform.localScale = new Vector3(webCamTextureMat.cols(), webCamTextureMat.rows(), 1);
            Debug.Log("Screen.width " + Screen.width + " Screen.height " + Screen.height + " Screen.orientation " + Screen.orientation);

            if (fpsMonitor != null)
            {
                fpsMonitor.Add("width", webCamTextureMat.width().ToString());
                fpsMonitor.Add("height", webCamTextureMat.height().ToString());
                fpsMonitor.Add("orientation", Screen.orientation.ToString());
            }


            float width = webCamTextureMat.width();
            float height = webCamTextureMat.height();

            float imageSizeScale = 1.0f;
            float widthScale = (float)Screen.width / width;
            float heightScale = (float)Screen.height / height;
            if (widthScale < heightScale)
            {
                Camera.main.orthographicSize = (width * (float)Screen.height / (float)Screen.width) / 2;
                imageSizeScale = (float)Screen.height / (float)Screen.width;
            }
            else
            {
                Camera.main.orthographicSize = height / 2;
            }

            // Camera.main.fieldOfView = 176;

            // set camera parameters.
            double fx;
            double fy;
            double cx;
            double cy;

            string loadDirectoryPath = Path.Combine(Application.persistentDataPath, "ArUcoCameraCalibrationExample");
            string calibratonDirectoryName = "camera_parameters" + width + "x" + height;
            string loadCalibratonFileDirectoryPath = Path.Combine(loadDirectoryPath, calibratonDirectoryName);
            string loadPath = Path.Combine(loadCalibratonFileDirectoryPath, calibratonDirectoryName + ".xml");
            if (useStoredCameraParameters && File.Exists(loadPath))
            {
                CameraParameters param;
                XmlSerializer serializer = new XmlSerializer(typeof(CameraParameters));
                using (var stream = new FileStream(loadPath, FileMode.Open))
                {
                    param = (CameraParameters)serializer.Deserialize(stream);
                }

                camMatrix = param.GetCameraMatrix();
                distCoeffs = new MatOfDouble(param.GetDistortionCoefficients());

                fx = param.camera_matrix[0];
                fy = param.camera_matrix[4];
                cx = param.camera_matrix[2];
                cy = param.camera_matrix[5];

                Debug.Log("Loaded CameraParameters from a stored XML file.");
                Debug.Log("loadPath: " + loadPath);
            }
            else
            {
                int max_d = (int)Mathf.Max(width, height);
                fx = max_d;
                fy = max_d;
                cx = width / 2.0f;
                cy = height / 2.0f;

                camMatrix = new Mat(3, 3, CvType.CV_64FC1);
                camMatrix.put(0, 0, fx);
                camMatrix.put(0, 1, 0);
                camMatrix.put(0, 2, cx);
                camMatrix.put(1, 0, 0);
                camMatrix.put(1, 1, fy);
                camMatrix.put(1, 2, cy);
                camMatrix.put(2, 0, 0);
                camMatrix.put(2, 1, 0);
                camMatrix.put(2, 2, 1.0f);

                distCoeffs = new MatOfDouble(0, 0, 0, 0);
                Debug.Log("Created a dummy CameraParameters.");
            }
            Debug.Log("camMatrix " + camMatrix.dump());
            Debug.Log("distCoeffs " + distCoeffs.dump());

            // calibration camera matrix values.
            Size imageSize = new Size(width * imageSizeScale, height * imageSizeScale);
            double apertureWidth = 0;
            double apertureHeight = 0;
            double[] fovx = new double[1];
            double[] fovy = new double[1];
            double[] focalLength = new double[1];
            Point principalPoint = new Point(0, 0);
            double[] aspectratio = new double[1];

            Calib3d.calibrationMatrixValues(camMatrix, imageSize, apertureWidth, apertureHeight, fovx, fovy, focalLength, principalPoint, aspectratio);

            Debug.Log("imageSize " + imageSize.ToString());
            Debug.Log("apertureWidth " + apertureWidth);
            Debug.Log("apertureHeight " + apertureHeight);
            Debug.Log("fovx " + fovx[0]);
            Debug.Log("fovy " + fovy[0]);
            Debug.Log("focalLength " + focalLength[0]);
            Debug.Log("principalPoint " + principalPoint.ToString());
            Debug.Log("aspectratio " + aspectratio[0]);

            // To convert the difference of the FOV value of the OpenCV and Unity. 
            double fovXScale = (2.0 * Mathf.Atan((float)(imageSize.width / (2.0 * fx)))) / (Mathf.Atan2((float)cx, (float)fx) + Mathf.Atan2((float)(imageSize.width - cx), (float)fx));
            double fovYScale = (2.0 * Mathf.Atan((float)(imageSize.height / (2.0 * fy)))) / (Mathf.Atan2((float)cy, (float)fy) + Mathf.Atan2((float)(imageSize.height - cy), (float)fy));

            Debug.Log("fovXScale " + fovXScale);
            Debug.Log("fovYScale " + fovYScale);

            // Adjust Unity Camera FOV https://github.com/opencv/opencv/commit/8ed1945ccd52501f5ab22bdec6aa1f91f1e2cfd4
            if (widthScale < heightScale)
            {
                arCamera.fieldOfView = (float)(fovx[0] * fovXScale);
            }
            else
            {
                arCamera.fieldOfView = (float)(fovy[0] * fovYScale);
            }
            // Display objects near the camera.
            arCamera.nearClipPlane = 0.01f;

            rgbMat = new Mat(webCamTextureMat.rows(), webCamTextureMat.cols(), CvType.CV_8UC3);
            ids = new Mat();
            corners = new List<Mat>();
            rejectedCorners = new List<Mat>();
            rvecs = new Mat();
            tvecs = new Mat();
            rotMat = new Mat(3, 3, CvType.CV_64FC1);
            recoveredIdxs = new Mat();

            detectorParams = DetectorParameters.create();
            detectorParams.set_cornerRefinementMethod(Aruco.CORNER_REFINE_APRILTAG);
            dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_100);

            // Custom board creation
            // List<Mat> object1ObjPoints = new List<Mat>(){
            //     new MatOfPoint3f(new Point3(0.0, 0.006000000052154064, 0.11860000342130661), new Point3(0.0, 0.03500000014901161, 0.11860000342130661), new Point3(0.0, 0.03500000014901161, 0.08960000425577164), new Point3(0.0, 0.006000000052154064, 0.08960000425577164)),
            //     new MatOfPoint3f(new Point3(0.0, 0.040800001472234726, 0.11860000342130661), new Point3(0.0, 0.0697999969124794, 0.11860000342130661), new Point3(0.0, 0.0697999969124794, 0.08960000425577164), new Point3(0.0, 0.040800001472234726, 0.08960000425577164)),
            //     new MatOfPoint3f(new Point3(0.0, 0.07559999823570251, 0.11860000342130661), new Point3(0.0, 0.10459999740123749, 0.11860000342130661), new Point3(0.0, 0.10459999740123749, 0.08960000425577164), new Point3(0.0, 0.07559999823570251, 0.08960000425577164)),
            //     new MatOfPoint3f(new Point3(0.0, 0.1103999987244606, 0.11860000342130661), new Point3(0.0, 0.13939999043941498, 0.11860000342130661), new Point3(0.0, 0.13939999043941498, 0.08960000425577164), new Point3(0.0, 0.1103999987244606, 0.08960000425577164)),
            //     new MatOfPoint3f(new Point3(0.0, 0.1451999992132187, 0.11860000342130661), new Point3(0.0, 0.17419999837875366, 0.11860000342130661), new Point3(0.0, 0.17419999837875366, 0.08960000425577164), new Point3(0.0, 0.1451999992132187, 0.08960000425577164)),
            //     new MatOfPoint3f(new Point3(0.0, 0.17999999225139618, 0.11860000342130661), new Point3(0.0, 0.20899999141693115, 0.11860000342130661), new Point3(0.0, 0.20899999141693115, 0.08960000425577164), new Point3(0.0, 0.17999999225139618, 0.08960000425577164)),
            //     new MatOfPoint3f(new Point3(0.0, 0.21480000019073486, 0.11860000342130661), new Point3(0.0, 0.24379999935626984, 0.11860000342130661), new Point3(0.0, 0.24379999935626984, 0.08960000425577164), new Point3(0.0, 0.21480000019073486, 0.08960000425577164)),
            //     new MatOfPoint3f(new Point3(0.0, 0.006000000052154064, 0.08380000293254852), new Point3(0.0, 0.03500000014901161, 0.08380000293254852), new Point3(0.0, 0.03500000014901161, 0.05480000004172325), new Point3(0.0, 0.006000000052154064, 0.05480000004172325)),
            //     new MatOfPoint3f(new Point3(0.0, 0.040800001472234726, 0.08380000293254852), new Point3(0.0, 0.0697999969124794, 0.08380000293254852), new Point3(0.0, 0.0697999969124794, 0.05480000004172325), new Point3(0.0, 0.040800001472234726, 0.05480000004172325)),
            //     new MatOfPoint3f(new Point3(0.0, 0.07559999823570251, 0.08380000293254852), new Point3(0.0, 0.10459999740123749, 0.08380000293254852), new Point3(0.0, 0.10459999740123749, 0.05480000004172325), new Point3(0.0, 0.07559999823570251, 0.05480000004172325)),
            //     new MatOfPoint3f(new Point3(0.0, 0.1103999987244606, 0.08380000293254852), new Point3(0.0, 0.13939999043941498, 0.08380000293254852), new Point3(0.0, 0.13939999043941498, 0.05480000004172325), new Point3(0.0, 0.1103999987244606, 0.05480000004172325)),
            //     new MatOfPoint3f(new Point3(0.0, 0.1451999992132187, 0.08380000293254852), new Point3(0.0, 0.17419999837875366, 0.08380000293254852), new Point3(0.0, 0.17419999837875366, 0.05480000004172325), new Point3(0.0, 0.1451999992132187, 0.05480000004172325)),
            //     new MatOfPoint3f(new Point3(0.0, 0.17999999225139618, 0.08380000293254852), new Point3(0.0, 0.20899999141693115, 0.08380000293254852), new Point3(0.0, 0.20899999141693115, 0.05480000004172325), new Point3(0.0, 0.17999999225139618, 0.05480000004172325)),
            //     new MatOfPoint3f(new Point3(0.0, 0.21480000019073486, 0.08380000293254852), new Point3(0.0, 0.24379999935626984, 0.08380000293254852), new Point3(0.0, 0.24379999935626984, 0.05480000004172325), new Point3(0.0, 0.21480000019073486, 0.05480000004172325)),
            //     new MatOfPoint3f(new Point3(0.0, 0.006000000052154064, 0.04899999871850014), new Point3(0.0, 0.03500000014901161, 0.04899999871850014), new Point3(0.0, 0.03500000014901161, 0.019999999552965164), new Point3(0.0, 0.006000000052154064, 0.019999999552965164)),
            //     new MatOfPoint3f(new Point3(0.0, 0.040800001472234726, 0.04899999871850014), new Point3(0.0, 0.0697999969124794, 0.04899999871850014), new Point3(0.0, 0.0697999969124794, 0.019999999552965164), new Point3(0.0, 0.040800001472234726, 0.019999999552965164)),
            //     new MatOfPoint3f(new Point3(0.0, 0.07559999823570251, 0.04899999871850014), new Point3(0.0, 0.10459999740123749, 0.04899999871850014), new Point3(0.0, 0.10459999740123749, 0.019999999552965164), new Point3(0.0, 0.07559999823570251, 0.019999999552965164)),
            //     new MatOfPoint3f(new Point3(0.0, 0.1103999987244606, 0.04899999871850014), new Point3(0.0, 0.13939999043941498, 0.04899999871850014), new Point3(0.0, 0.13939999043941498, 0.019999999552965164), new Point3(0.0, 0.1103999987244606, 0.019999999552965164)),
            //     new MatOfPoint3f(new Point3(0.0, 0.1451999992132187, 0.04899999871850014), new Point3(0.0, 0.17419999837875366, 0.04899999871850014), new Point3(0.0, 0.17419999837875366, 0.019999999552965164), new Point3(0.0, 0.1451999992132187, 0.019999999552965164)),
            //     new MatOfPoint3f(new Point3(0.0, 0.17999999225139618, 0.04899999871850014), new Point3(0.0, 0.20899999141693115, 0.04899999871850014), new Point3(0.0, 0.20899999141693115, 0.019999999552965164), new Point3(0.0, 0.17999999225139618, 0.019999999552965164)),
            //     new MatOfPoint3f(new Point3(0.0, 0.21480000019073486, 0.04899999871850014), new Point3(0.0, 0.24379999935626984, 0.04899999871850014), new Point3(0.0, 0.24379999935626984, 0.019999999552965164), new Point3(0.0, 0.21480000019073486, 0.019999999552965164)),
            //     new MatOfPoint3f(new Point3(0.024500001221895218, 0.006000000052154064, 0.0), new Point3(0.024500001221895218, 0.03500000014901161, 0.0), new Point3(0.05350000038743019, 0.03500000014901161, 0.0), new Point3(0.05350000038743019, 0.006000000052154064, 0.0)),
            //     new MatOfPoint3f(new Point3(0.024500001221895218, 0.040800001472234726, 0.0), new Point3(0.024500001221895218, 0.0697999969124794, 0.0), new Point3(0.05350000038743019, 0.0697999969124794, 0.0), new Point3(0.05350000038743019, 0.040800001472234726, 0.0)),
            //     new MatOfPoint3f(new Point3(0.024500001221895218, 0.07559999823570251, 0.0), new Point3(0.024500001221895218, 0.10459999740123749, 0.0), new Point3(0.05350000038743019, 0.10459999740123749, 0.0), new Point3(0.05350000038743019, 0.07559999823570251, 0.0)),
            //     new MatOfPoint3f(new Point3(0.024500001221895218, 0.1103999987244606, 0.0), new Point3(0.024500001221895218, 0.13939999043941498, 0.0), new Point3(0.05350000038743019, 0.13939999043941498, 0.0), new Point3(0.05350000038743019, 0.1103999987244606, 0.0)),
            //     new MatOfPoint3f(new Point3(0.024500001221895218, 0.1451999992132187, 0.0), new Point3(0.024500001221895218, 0.17419999837875366, 0.0), new Point3(0.05350000038743019, 0.17419999837875366, 0.0), new Point3(0.05350000038743019, 0.1451999992132187, 0.0)),
            //     new MatOfPoint3f(new Point3(0.024500001221895218, 0.17999999225139618, 0.0), new Point3(0.024500001221895218, 0.20899999141693115, 0.0), new Point3(0.05350000038743019, 0.20899999141693115, 0.0), new Point3(0.05350000038743019, 0.17999999225139618, 0.0)),
            //     new MatOfPoint3f(new Point3(0.024500001221895218, 0.21480000019073486, 0.0), new Point3(0.024500001221895218, 0.24379999935626984, 0.0), new Point3(0.05350000038743019, 0.24379999935626984, 0.0), new Point3(0.05350000038743019, 0.21480000019073486, 0.0)),
            // };

            // object1Ids = new MatOfInt(new int[] {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, });
            // object1Board = Board.create(object1ObjPoints, dictionary, object1Ids);


            object1 = new TrackedObject(arGameObject1, new TPiece().objPoints, dictionary, new TPiece().ids, detectorParams);
            object2 = new TrackedObject(arGameObject2, new Welder().objPoints, dictionary, new Welder().ids, detectorParams);



            // List<Mat> object2ObjPoints = new List<Mat>(){
            //     new MatOfPoint3f(new Point3(-0.006000000052154064, 0.006000000052154064, 0.00800000037997961), new Point3(0.006000000052154064, 0.006000000052154064, 0.00800000037997961), new Point3(0.006000000052154064, -0.006000000052154064, 0.00800000037997961), new Point3(-0.006000000052154064, -0.006000000052154064, 0.00800000037997961)),
            //     new MatOfPoint3f(new Point3(0.010000000707805157, 0.006000000052154064, 0.00800000037997961), new Point3(0.02200000174343586, 0.006000000052154064, 0.00800000037997961), new Point3(0.02200000174343586, -0.006000000052154064, 0.00800000037997961), new Point3(0.010000000707805157, -0.006000000052154064, 0.00800000037997961)),
            //     new MatOfPoint3f(new Point3(0.026000002399086952, 0.006000000052154064, 0.00800000037997961), new Point3(0.03799999877810478, 0.006000000052154064, 0.00800000037997961), new Point3(0.03799999877810478, -0.006000000052154064, 0.00800000037997961), new Point3(0.026000002399086952, -0.006000000052154064, 0.00800000037997961)),
            //     new MatOfPoint3f(new Point3(0.041999999433755875, 0.006000000052154064, 0.00800000037997961), new Point3(0.05400000140070915, 0.006000000052154064, 0.00800000037997961), new Point3(0.05400000140070915, -0.006000000052154064, 0.00800000037997961), new Point3(0.041999999433755875, -0.006000000052154064, 0.00800000037997961)),
            //     new MatOfPoint3f(new Point3(-0.006000000052154064, -0.00800000037997961, 0.006000000052154064), new Point3(0.006000000052154064, -0.00800000037997961, 0.006000000052154064), new Point3(0.006000000052154064, -0.00800000037997961, -0.006000000052154064), new Point3(-0.006000000052154064, -0.00800000037997961, -0.006000000052154064)),
            //     new MatOfPoint3f(new Point3(0.010000000707805157, -0.00800000037997961, 0.006000000052154064), new Point3(0.02200000174343586, -0.00800000037997961, 0.006000000052154064), new Point3(0.02200000174343586, -0.00800000037997961, -0.006000000052154064), new Point3(0.010000000707805157, -0.00800000037997961, -0.006000000052154064)),
            //     new MatOfPoint3f(new Point3(0.026000002399086952, -0.00800000037997961, 0.006000000052154064), new Point3(0.03799999877810478, -0.00800000037997961, 0.006000000052154064), new Point3(0.03799999877810478, -0.00800000037997961, -0.006000000052154064), new Point3(0.026000002399086952, -0.00800000037997961, -0.006000000052154064)),
            //     new MatOfPoint3f(new Point3(0.041999999433755875, -0.00800000037997961, 0.006000000052154064), new Point3(0.05400000140070915, -0.00800000037997961, 0.006000000052154064), new Point3(0.05400000140070915, -0.00800000037997961, -0.006000000052154064), new Point3(0.041999999433755875, -0.00800000037997961, -0.006000000052154064)),
            //     new MatOfPoint3f(new Point3(-0.006000000052154064, -0.006000000052154064, -0.00800000037997961), new Point3(0.006000000052154064, -0.006000000052154064, -0.00800000037997961), new Point3(0.006000000052154064, 0.006000000052154064, -0.00800000037997961), new Point3(-0.006000000052154064, 0.006000000052154064, -0.00800000037997961)),
            //     new MatOfPoint3f(new Point3(0.010000000707805157, -0.006000000052154064, -0.00800000037997961), new Point3(0.02200000174343586, -0.006000000052154064, -0.00800000037997961), new Point3(0.02200000174343586, 0.006000000052154064, -0.00800000037997961), new Point3(0.010000000707805157, 0.006000000052154064, -0.00800000037997961)),
            //     new MatOfPoint3f(new Point3(0.026000002399086952, -0.006000000052154064, -0.00800000037997961), new Point3(0.03799999877810478, -0.006000000052154064, -0.00800000037997961), new Point3(0.03799999877810478, 0.006000000052154064, -0.00800000037997961), new Point3(0.026000002399086952, 0.006000000052154064, -0.00800000037997961)),
            //     new MatOfPoint3f(new Point3(0.041999999433755875, -0.006000000052154064, -0.00800000037997961), new Point3(0.05400000140070915, -0.006000000052154064, -0.00800000037997961), new Point3(0.05400000140070915, 0.006000000052154064, -0.00800000037997961), new Point3(0.041999999433755875, 0.006000000052154064, -0.00800000037997961)),
            //     new MatOfPoint3f(new Point3(-0.006000000052154064, 0.00800000037997961, -0.006000000052154064), new Point3(0.006000000052154064, 0.00800000037997961, -0.006000000052154064), new Point3(0.006000000052154064, 0.00800000037997961, 0.006000000052154064), new Point3(-0.006000000052154064, 0.00800000037997961, 0.006000000052154064)),
            //     new MatOfPoint3f(new Point3(0.010000000707805157, 0.00800000037997961, -0.006000000052154064), new Point3(0.02200000174343586, 0.00800000037997961, -0.006000000052154064), new Point3(0.02200000174343586, 0.00800000037997961, 0.006000000052154064), new Point3(0.010000000707805157, 0.00800000037997961, 0.006000000052154064)),
            //     new MatOfPoint3f(new Point3(0.026000002399086952, 0.00800000037997961, -0.006000000052154064), new Point3(0.03799999877810478, 0.00800000037997961, -0.006000000052154064), new Point3(0.03799999877810478, 0.00800000037997961, 0.006000000052154064), new Point3(0.026000002399086952, 0.00800000037997961, 0.006000000052154064)),
            //     new MatOfPoint3f(new Point3(0.041999999433755875, 0.00800000037997961, -0.006000000052154064), new Point3(0.05400000140070915, 0.00800000037997961, -0.006000000052154064), new Point3(0.05400000140070915, 0.00800000037997961, 0.006000000052154064), new Point3(0.041999999433755875, 0.00800000037997961, 0.006000000052154064)),
            //     new MatOfPoint3f(new Point3(0.0560000017285347, 0.006000000052154064, -0.006000000052154064), new Point3(0.0560000017285347, -0.006000000052154064, -0.006000000052154064), new Point3(0.0560000017285347, -0.006000000052154064, 0.006000000052154064), new Point3(0.0560000017285347, 0.006000000052154064, 0.006000000052154064)),
            // };
            // object2Ids = new MatOfInt(new int[] {60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, });
            // object2Board = Board.create(object2ObjPoints, dictionary, object2Ids);

            // if WebCamera is frontFaceing, flip Mat.
            webCamTextureToMatHelper.flipHorizontal = webCamTextureToMatHelper.GetWebCamDevice().isFrontFacing;
        }

        /// <summary>
        /// Raises the webcam texture to mat helper disposed event.
        /// </summary>
        public void OnWebCamTextureToMatHelperDisposed()
        {
            Debug.Log("OnWebCamTextureToMatHelperDisposed");
            if (rgbMat != null)
                rgbMat.Dispose();
            if (texture != null)
            {
                Texture2D.Destroy(texture);
                texture = null;
            }
            if (ids != null)
                ids.Dispose();
            foreach (var item in corners)
            {
                item.Dispose();
            }
            corners.Clear();
            foreach (var item in rejectedCorners)
            {
                item.Dispose();
            }
            rejectedCorners.Clear();
            if (rvecs != null)
                rvecs.Dispose();
            if (tvecs != null)
                tvecs.Dispose();
            if (rotMat != null)
                rotMat.Dispose();
            if (recoveredIdxs != null)
                recoveredIdxs.Dispose();
            if (object1Board != null)
                object1Board.Dispose();
            if (object2Board != null)
                object2Board.Dispose();
        }

        /// <summary>
        /// Raises the webcam texture to mat helper error occurred event.
        /// </summary>
        /// <param name="errorCode">Error code.</param>
        public void OnWebCamTextureToMatHelperErrorOccurred(WebCamTextureToMatHelper.ErrorCode errorCode)
        {
            Debug.Log("OnWebCamTextureToMatHelperErrorOccurred " + errorCode);
        }

        // Update is called once per frame
        void Update()
        {
            if (webCamTextureToMatHelper.IsPlaying() && webCamTextureToMatHelper.DidUpdateThisFrame())
            {
                background.rectTransform.localScale = new Vector3(-1f, 1f, 1f);

                Mat rgbaMat = webCamTextureToMatHelper.GetMat();
                Imgproc.cvtColor(rgbaMat, rgbMat, Imgproc.COLOR_RGBA2RGB);

                // detect markers.
                Aruco.detectMarkers(rgbMat, dictionary, corners, ids, detectorParams, rejectedCorners, camMatrix, distCoeffs);

                // if (refineMarkerDetection)
                // {
                //     Aruco.refineDetectedMarkers(rgbMat, customBoard, corners, ids, rejectedCorners, camMatrix, distCoeffs, 10f, 3f, true, recoveredIdxs, detectorParams);
                // }

                // if at least one marker detected
                if (ids.total() > 0)
                {
                    // draw markers.

                    if (showDetectedMarkers)
                    {
                        Aruco.drawDetectedMarkers(rgbMat, corners, ids, new Scalar(0, 255, 0));
                    }
                    if (applyEstimationPose)
                    {
                        object1.EstimatePose(rgbMat, camMatrix, arCamera, recoveredIdxs, corners, rejectedCorners, ids);
                        object2.EstimatePose(rgbMat, camMatrix, arCamera, recoveredIdxs, corners, rejectedCorners, ids);
                    }
                }

                if (showRejectedCorners && rejectedCorners.Count > 0)
                {
                    Aruco.drawDetectedMarkers(rgbMat, rejectedCorners, new Mat(), new Scalar(255, 0, 0));
                }

                //Imgproc.putText (rgbaMat, "W:" + rgbaMat.width () + " H:" + rgbaMat.height () + " SO:" + Screen.orientation, new Point (5, rgbaMat.rows () - 10), Imgproc.FONT_HERSHEY_SIMPLEX, 1.0, new Scalar (255, 255, 255, 255), 2, Imgproc.LINE_AA, false);

                Imgproc.cvtColor(rgbMat, rgbaMat, Imgproc.COLOR_RGB2RGBA);
                Utils.fastMatToTexture2D(rgbaMat, texture);
            }
        }

        private void ResetObjectTransform()
        {
            // reset AR object transform.
            Matrix4x4 i = Matrix4x4.identity;
            ARUtils.SetTransformFromMatrix(arCamera.transform, ref i);
            ARUtils.SetTransformFromMatrix(arGameObject1.transform, ref i);
            ARUtils.SetTransformFromMatrix(arGameObject2.transform, ref i);
        }

        /// <summary>
        /// Raises the destroy event.
        /// </summary>
        void OnDestroy()
        {
            webCamTextureToMatHelper.Dispose();
        }

        /// <summary>
        /// Raises the back button click event.
        /// </summary>
        public void OnBackButtonClick()
        {
            SceneManager.LoadScene("OpenCVForUnityExample");
        }

        /// <summary>
        /// Raises the play button click event.
        /// </summary>
        public void OnPlayButtonClick()
        {
            webCamTextureToMatHelper.Play();
        }

        /// <summary>
        /// Raises the pause button click event.
        /// </summary>
        public void OnPauseButtonClick()
        {
            webCamTextureToMatHelper.Pause();
        }

        /// <summary>
        /// Raises the stop button click event.
        /// </summary>
        public void OnStopButtonClick()
        {
            webCamTextureToMatHelper.Stop();
        }

        /// <summary>
        /// Raises the change camera button click event.
        /// </summary>
        public void OnChangeCameraButtonClick()
        {
            webCamTextureToMatHelper.requestedIsFrontFacing = !webCamTextureToMatHelper.IsFrontFacing();
        }
    }
}
