#include <kinect_run/KinectFrame.h>
#include <kinect_run/KinectRun.h>

KinectFrame::KinectFrame(KinectRun *kr) : _kr(kr){
    //k4a_calibration_t currCalibration;

    k4a_device_get_calibration(_kr->_device, K4A_DEPTH_MODE_NFOV_UNBINNED, K4A_COLOR_RESOLUTION_1080P, &_calibration);
}

KinectFrame::~KinectFrame()
{
    k4a_capture_release(_capture);
    k4abt_frame_release(_bodyFrame);
    _colorDepthImage.reset();
    _colorImage.reset();
    _depthImage.reset();
    _xyzImage.reset();
}

void KinectFrame::getSensorCapture()
{
    k4a_wait_result_t get_capture_result = k4a_device_get_capture(_kr->_device, &_capture, K4A_WAIT_INFINITE);
    if (get_capture_result == K4A_WAIT_RESULT_SUCCEEDED)
    {
        printf("Device capture successful\n");
    }
    else if (get_capture_result == K4A_WAIT_RESULT_TIMEOUT)
    {
        // It should never hit time out when K4A_WAIT_INFINITE is set.
        printf("Error! Get depth frame time out!\n");
        return;
    }
    else
    {
        printf("Get depth capture returned error: %d\n", get_capture_result);
        return;
    }
}

void KinectFrame::bodyFrameCapture()
{
    if (_capture == NULL)
    {
        printf("No sensor capture present. Exiting\n");
        return;
    }


    k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(_kr->tracker, _capture, K4A_WAIT_INFINITE);

    // Check result is returned
    if (queue_capture_result == K4A_WAIT_RESULT_TIMEOUT)
    {
        // It should never hit timeout when K4A_WAIT_INFINITE is set.
        printf("Error! Add capture to tracker process queue timeout!\n");
        return;
    }
    else if (queue_capture_result == K4A_WAIT_RESULT_FAILED)
    {
        printf("Error! Add capture to tracker process queue failed!\n");
        return;
    }

    //k4abt_frame_t body_frame = NULL;
    // Get Frame from body tracker frame
    k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(_kr->tracker, &_bodyFrame, K4A_WAIT_INFINITE);
    /**
    if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED)
    {
        // Successfully popped the body tracking result. Start your processing

        size_t num_bodies = k4abt_frame_get_num_bodies(body_frame);
        uint32_t bodyID = k4abt_frame_get_body_id(body_frame, 0);
        k4a_image_t bodyIndexMap = k4abt_frame_get_body_index_map(body_frame);

        k4a_image_release(bodyIndexMap);
        printf("%zu bodies are detected\n", num_bodies);
        return;
    } **/
    if (pop_frame_result == K4A_WAIT_RESULT_TIMEOUT)
    {
        //  It should never hit timeout when K4A_WAIT_INFINITE is set.
        printf("Error! Pop body frame result timeout!\n");
        return;
    }
}

//Detects bodies and fills in vectors for joint positions, both 3d and 2d
void KinectFrame::locateJoints()
{
    if(_bodyFrame == NULL){
        printf("Body Frame is null. Exiting\n");
        return;
    }
    
    _num_bodies = k4abt_frame_get_num_bodies(_bodyFrame);
    printf("Locating joints: %zu\n", _num_bodies);
    uint32_t bodyID = k4abt_frame_get_body_id(_bodyFrame, 0);
    k4a_image_t bodyIndexMap = k4abt_frame_get_body_index_map(_bodyFrame);
    

    k4a_float2_t xy;
    //k4a_calibration_t currCalibration;
    int k;

    //k4a_device_get_calibration(_kr->_device, K4A_DEPTH_MODE_NFOV_UNBINNED, K4A_COLOR_RESOLUTION_1080P, &currCalibration);

    //vector<k4a_float3_t> chest_positions;
    for (uint32_t i = 0; i < _num_bodies; i++)
    {
        k4abt_skeleton_t skeleton;
        k4abt_frame_get_body_skeleton(_bodyFrame, i, &skeleton);

        // vector<float> joint_pos(skeleton.joints[2].position.xyz.x, skeleton.joints[2].position.xyz.y);
        //chest_positions.push_back(skeleton.joints[K4ABT_JOINT_SPINE_CHEST].position);
        printf("Chest position detected at: %f, %f, %f\n", skeleton.joints[K4ABT_JOINT_SPINE_CHEST].position.xyz.x, skeleton.joints[K4ABT_JOINT_SPINE_CHEST].position.xyz.y, skeleton.joints[K4ABT_JOINT_SPINE_CHEST].position.xyz.z);
        _chestXYZ.push_back(skeleton.joints[K4ABT_JOINT_SPINE_CHEST].position);

        _landmarkJoint = skeleton.joints[K4ABT_JOINT_SPINE_CHEST];

        k4a_calibration_3d_to_2d(&_calibration, &skeleton.joints[K4ABT_JOINT_SPINE_CHEST].position, K4A_CALIBRATION_TYPE_DEPTH, 
                                K4A_CALIBRATION_TYPE_COLOR, &xy, &k);
        _jointsXY.push_back(xy);
    }

    // convertImage(bodyIndexMap, CV_8UC1);
    // drawJoints(bodyIndexMap, CV_8UC1, chest_positions);
    k4a_image_release(bodyIndexMap);
    printf("%zu bodies are detected\n", _num_bodies);
}

cv::Mat KinectFrame::getColorImage(){
    k4a_image_t image = k4a_capture_get_color_image(_capture);

    uint8_t* imgBuffer = k4a_image_get_buffer(image);
    int height = k4a_image_get_height_pixels(image);
    int width = k4a_image_get_width_pixels(image);

    cv::Mat retImage(height, width, CV_8UC4, (void*) imgBuffer, cv::Mat::AUTO_STEP);


    //cv::imshow("test", retImage);
    //cvWaitKey(30);

    return retImage;
}

void KinectFrame::getAllImages(){
    _colorImage = k4a_capture_get_color_image(_capture);
    _depthImage = k4a_capture_get_depth_image(_capture);

    _xyzImage =
        k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM,
            _depthImage.get_width_pixels(), _depthImage.get_height_pixels(),
            _depthImage.get_width_pixels() * 3 * sizeof(int16_t));
            
    _colorDepthImage = k4a::image::create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
        _depthImage.get_width_pixels(), _depthImage.get_height_pixels(),
        _depthImage.get_width_pixels() * 4 * sizeof(int8_t));
}

cv::Mat KinectFrame::drawJointImage(){
    cv::Mat colorImage = getColorImage();

    locateJoints();

    for(k4a_float2_t joint : _jointsXY){
        cv::Point2f point(joint.xy.x, joint.xy.y);
        cv::circle(colorImage, point, 4, cv::Scalar(0, 0, 255), cv::FILLED);
    }

    return colorImage;
}

void KinectFrame::computeDepthInfo() {
    _kr->_transformation.depth_image_to_point_cloud(_depthImage, K4A_CALIBRATION_TYPE_DEPTH, &_xyzImage);
    _kr->_transformation.color_image_to_depth_camera(_depthImage, _colorImage, &_colorDepthImage);
}