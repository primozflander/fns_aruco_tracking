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
    public class TrackingManager : MonoBehaviour
    {
        public bool useStoredCameraParameters = false;
        public bool showRejectedCorners = false;
        public bool showDetectedMarkers = false;
        public bool applyEstimationPose = true;
        public bool refineMarkerDetection = true;
        public bool setAprilTagRefinementMethod = false;
        public bool enableLowPassFiltering = true;
        public bool shouldMoveARCamera = false;
        public bool disableWhenNotTracked = false;
        public GameObject arTPiece, arWelder, arStick;
        public Camera arCamera;
        WebCamTextureToMatHelper webCamTextureToMatHelper;
        Mat rgbMat;
        Mat camMatrix;
        Texture2D texture;
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
        TrackedObject object1, object2, object3;


    	void Awake() {
        	Screen.sleepTimeout = SleepTimeout.NeverSleep;
    	}


        void Start()
        {
            fpsMonitor = GetComponent<FpsMonitor>();
            webCamTextureToMatHelper = gameObject.GetComponent<WebCamTextureToMatHelper>();
            #if UNITY_ANDROID && !UNITY_EDITOR
            // Avoids the front camera low light issue that occurs in only some Android devices (e.g. Google Pixel, Pixel2).
            webCamTextureToMatHelper.avoidAndroidFrontCameraLowLightIssue = true;
            #endif
            webCamTextureToMatHelper.Initialize();
            arTPiece.SetActive(false);
            arWelder.SetActive(false);
            arStick.SetActive(false);
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
            gameObject.GetComponent<Renderer>().material.mainTexture = texture;
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
            dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_100);
            if (setAprilTagRefinementMethod) detectorParams.set_cornerRefinementMethod(Aruco.CORNER_REFINE_APRILTAG);

            // define tracked objects
            object1 = new TrackedObject(arTPiece, new TPiece().objPoints, dictionary, new TPiece().ids, detectorParams);
            object2 = new TrackedObject(arWelder, new Welder().objPoints, dictionary, new Welder().ids, detectorParams);
            object3 = new TrackedObject(arStick, new Stick().objPoints, dictionary, new Stick().ids, detectorParams);

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
                Mat rgbaMat = webCamTextureToMatHelper.GetMat();
                Imgproc.cvtColor(rgbaMat, rgbMat, Imgproc.COLOR_RGBA2RGB);

                // detect markers.
                Aruco.detectMarkers(rgbMat, dictionary, corners, ids, detectorParams, rejectedCorners, camMatrix, distCoeffs);

                object1.refineMarkerDetection = refineMarkerDetection;
                object2.refineMarkerDetection = refineMarkerDetection;
                object3.refineMarkerDetection = refineMarkerDetection;
                object1.enableLowPassFilter = enableLowPassFiltering;
                object2.enableLowPassFilter = enableLowPassFiltering;
                object3.enableLowPassFilter = enableLowPassFiltering;

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
                        object3.EstimatePose(rgbMat, camMatrix, arCamera, recoveredIdxs, corners, rejectedCorners, ids);
                    }
                    Debug.Log("detected!");
                }
                else
                {   
                    if (disableWhenNotTracked)
                    {
                        arTPiece.SetActive(false);
                        arWelder.SetActive(false);
                        arStick.SetActive(false);
                    }
                }

                if (showRejectedCorners && rejectedCorners.Count > 0)
                {
                    Aruco.drawDetectedMarkers(rgbMat, rejectedCorners, new Mat(), new Scalar(255, 0, 0));
                }

                Imgproc.cvtColor(rgbMat, rgbaMat, Imgproc.COLOR_RGB2RGBA);
                Utils.fastMatToTexture2D(rgbaMat, texture);
            }
        }

        private void ResetObjectTransform()
        {
            Matrix4x4 i = Matrix4x4.identity;
            ARUtils.SetTransformFromMatrix(arCamera.transform, ref i);
            ARUtils.SetTransformFromMatrix(arTPiece.transform, ref i);
            ARUtils.SetTransformFromMatrix(arWelder.transform, ref i);
            ARUtils.SetTransformFromMatrix(arStick.transform, ref i);
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

        public void OnEstimatePoseValueChanged ()
        {
            applyEstimationPose = !applyEstimationPose;
        }

        public void OnShowDetectedValueChanged ()
        {
            showDetectedMarkers = !showDetectedMarkers;
        }

        public void OnShowRejectedValueChanged ()
        {
            showRejectedCorners = !showRejectedCorners;
        }

        public void OnFilteringValueChanged ()
        {
            enableLowPassFiltering = !enableLowPassFiltering;
        }

        public void OnHideNotTrackedValueChanged ()
        {
            disableWhenNotTracked = !disableWhenNotTracked;
        }


    }
}
