#include <KinectFrame.h>

KinectFrame::~KinectFrame()
{
    k4a_capture_release(_capture);
    k4abt_frame_release(_bodyFrame);
}

KinectFrame::void getSensorCapture()
{
    k4a_wait_result_t get_capture_result = k4a_device_get_capture(_kr->device, &_capture, K4A_WAIT_INFINITE);
    if (get_capture_result == K4A_WAIT_RESULT_SUCCEEDED)
    {
        printf("Device capture successful\n");
    }
    else if (get_capture_result == K4A_WAIT_RESULT_TIMEOUT)
    {
        // It should never hit time out when K4A_WAIT_INFINITE is set.
        printf("Error! Get depth frame time out!\n");
        break;
    }
    else
    {
        printf("Get depth capture returned error: %d\n", get_capture_result);
        break;
    }
}

KinectFrame::void bodyFrameCapture()
{
    if (sensor_capture == NULL)
    {
        printf("No sensor capture present. Exiting\n");
        return;
    }

    k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(tracker, sensor_capture, K4A_WAIT_INFINITE);

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

    k4abt_frame_t body_frame = NULL;
    // Get Frame from body tracker frame
    k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(tracker, &body_frame, K4A_WAIT_INFINITE);
    if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED)
    {
        // Successfully popped the body tracking result. Start your processing

        size_t num_bodies = k4abt_frame_get_num_bodies(body_frame);
        uint32_t bodyID = k4abt_frame_get_body_id(body_frame, 0);
        k4a_image_t bodyIndexMap = k4abt_frame_get_body_index_map(body_frame);

        vector<k4a_float3_t> chest_positions;
        for (uint32_t i = 0; i < num_bodies; i++)
        {
            k4abt_skeleton_t skeleton;
            k4abt_frame_get_body_skeleton(body_frame, i, &skeleton);

            // vector<float> joint_pos(skeleton.joints[2].position.xyz.x, skeleton.joints[2].position.xyz.y);
            chest_positions.push_back(skeleton.joints[K4ABT_JOINT_SPINE_CHEST].position);
            printf("Chest position detected at: %f, %f, %f\n", skeleton.joints[K4ABT_JOINT_SPINE_CHEST].position.xyz.x, skeleton.joints[K4ABT_JOINT_SPINE_CHEST].position.xyz.y, skeleton.joints[K4ABT_JOINT_SPINE_CHEST].position.xyz.z);
            // chest_positions.push_back(joint_pos);
        }

        // convertImage(bodyIndexMap, CV_8UC1);
        // drawJoints(bodyIndexMap, CV_8UC1, chest_positions);
        k4a_image_release(bodyIndexMap);
        printf("%zu bodies are detected\n", num_bodies);
    }
    else if (pop_frame_result == K4A_WAIT_RESULT_TIMEOUT)
    {
        //  It should never hit timeout when K4A_WAIT_INFINITE is set.
        printf("Error! Pop body frame result timeout!\n");
        break;
    }
    else
    {
        printf("Pop body frame result failed!\n");
        break;
    }
}

KinectFrame::void locateJoints()
{
    if(_bodyFrame == NULL){
        printf("Body Frame is null. Exiting\n");
        return;
    }
    size_t num_bodies = k4abt_frame_get_num_bodies(_bodyFrame);
    uint32_t bodyID = k4abt_frame_get_body_id(_bodyFrame, 0);
    k4a_image_t bodyIndexMap = k4abt_frame_get_body_index_map(_bodyFrame);

    //vector<k4a_float3_t> chest_positions;
    for (uint32_t i = 0; i < num_bodies; i++)
    {
        k4abt_skeleton_t skeleton;
        k4abt_frame_get_body_skeleton(body_frame, i, &skeleton);

        // vector<float> joint_pos(skeleton.joints[2].position.xyz.x, skeleton.joints[2].position.xyz.y);
        //chest_positions.push_back(skeleton.joints[K4ABT_JOINT_SPINE_CHEST].position);
        printf("Chest position detected at: %f, %f, %f\n", skeleton.joints[K4ABT_JOINT_SPINE_CHEST].position.xyz.x, skeleton.joints[K4ABT_JOINT_SPINE_CHEST].position.xyz.y, skeleton.joints[K4ABT_JOINT_SPINE_CHEST].position.xyz.z);
        // chest_positions.push_back(joint_pos);
    }

    // convertImage(bodyIndexMap, CV_8UC1);
    // drawJoints(bodyIndexMap, CV_8UC1, chest_positions);
    k4a_image_release(bodyIndexMap);
    printf("%zu bodies are detected\n", num_bodies);
}

k4a_capture_t sensor_capture;

k4a_wait_result_t get_capture_result = k4a_device_get_capture(device, &sensor_capture, K4A_WAIT_INFINITE);
if (get_capture_result == K4A_WAIT_RESULT_SUCCEEDED)
{
    frame_count++;
    k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(tracker, sensor_capture, K4A_WAIT_INFINITE);

    // Get Depth image
    k4a_image_t image = k4a_capture_get_depth_image(sensor_capture); // CREATE IMAGE FROM SENSOR CAPTURE
    if (image != NULL)

        // printf("Successfully capture image data: %d by %d \n", k4a_image_get_height_pixels(image), k4a_image_get_width_pixels(image));
        // convertImage(image);

        k4a_image_release(image);

    k4a_capture_release(sensor_capture); // Remember to release the sensor capture once you finish using it
    if (queue_capture_result == K4A_WAIT_RESULT_TIMEOUT)
    {
        // It should never hit timeout when K4A_WAIT_INFINITE is set.
        printf("Error! Add capture to tracker process queue timeout!\n");
        break;
    }
    else if (queue_capture_result == K4A_WAIT_RESULT_FAILED)
    {
        printf("Error! Add capture to tracker process queue failed!\n");
        break;
    }

    k4abt_frame_t body_frame = NULL;
    // Get Frame from body tracker frame
    k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(tracker, &body_frame, K4A_WAIT_INFINITE);
    if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED)
    {
        // Successfully popped the body tracking result. Start your processing

        size_t num_bodies = k4abt_frame_get_num_bodies(body_frame);
        uint32_t bodyID = k4abt_frame_get_body_id(body_frame, 0);
        k4a_image_t bodyIndexMap = k4abt_frame_get_body_index_map(body_frame);

        vector<k4a_float3_t> chest_positions;
        for (uint32_t i = 0; i < num_bodies; i++)
        {
            k4abt_skeleton_t skeleton;
            k4abt_frame_get_body_skeleton(body_frame, i, &skeleton);

            // vector<float> joint_pos(skeleton.joints[2].position.xyz.x, skeleton.joints[2].position.xyz.y);
            chest_positions.push_back(skeleton.joints[K4ABT_JOINT_SPINE_CHEST].position);
            printf("Chest position detected at: %f, %f, %f\n", skeleton.joints[K4ABT_JOINT_SPINE_CHEST].position.xyz.x, skeleton.joints[K4ABT_JOINT_SPINE_CHEST].position.xyz.y, skeleton.joints[K4ABT_JOINT_SPINE_CHEST].position.xyz.z);
            // chest_positions.push_back(joint_pos);
        }

        // convertImage(bodyIndexMap, CV_8UC1);
        // drawJoints(bodyIndexMap, CV_8UC1, chest_positions);
        k4a_image_release(bodyIndexMap);
        printf("%zu bodies are detected\n", num_bodies);

        // DO STUFF********************************************************

        k4abt_frame_release(body_frame); // Remember to release the body frame once you finish using it
    }
    else if (pop_frame_result == K4A_WAIT_RESULT_TIMEOUT)
    {
        //  It should never hit timeout when K4A_WAIT_INFINITE is set.
        printf("Error! Pop body frame result timeout!\n");
        break;
    }
    else
    {
        printf("Pop body frame result failed!\n");
        break;
    }
}
else if (get_capture_result == K4A_WAIT_RESULT_TIMEOUT)
{
    // It should never hit time out when K4A_WAIT_INFINITE is set.
    printf("Error! Get depth frame time out!\n");
    break;
}
else
{
    printf("Get depth capture returned error: %d\n", get_capture_result);
    break;
}
}