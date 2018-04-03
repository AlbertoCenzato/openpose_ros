#include <openpose_ros_io.h>

#define NUM_BODY_PARTS_COCO 18
#define NUM_BODY_PARTS_MPI_15
#define NUM_FACE_PARTS 70
#define NUM_HAND_PARTS 21

#define NUM_BODY_PARTS NUM_BODY_PARTS_COCO

#define HUMAN_DATA_SIZE NUM_BODY_PARTS*3
#define FACE_DATA_SIZE  NUM_FACE_PARTS*3
#define HAND_DATA_SIZE  NUM_HAND_PARTS*3

enum class Hand {
    left  = 0,
    right = 1
};

using namespace openpose_ros;

/**
 * Extracts body_key_points_with_prob and num_body_key_points_with_non_zero_prob from Datum into OpenPoseHuman msg
 * @param input OpenPose op::Datum
 * @param output human message
 * @param index of the human inside Datum
 */
void getBodyKeyPoints(const op::Datum &datum,  openpose_ros_msgs::OpenPoseHuman &human, int humanID);

/**
 * Extracts face_key_points_with_prob, num_face_key_points_with_non_zero_prob
 * and face_bounding_box from Datum into OpenPoseHuman msg
 * @param input OpenPose op::Datum
 * @param output human message
 * @param index of the human inside Datum
 */
void getFaceKeyPoints(const op::Datum &datum,  openpose_ros_msgs::OpenPoseHuman &human, int humanID);

/**
 * Extracts num_left_hand_key_points_with_non_zero_prob, left_hand_bounding_box
 * and left_hand_key_points_with_prob from Datum into OpenPoseHuman msg
 * @param input OpenPose op::Datum
 * @param output human message
 * @param index of the human inside Datum
 */
void getLeftHandKeyPoints(const op::Datum &datum,  openpose_ros_msgs::OpenPoseHuman &human, int humanID);

/**
 * Extracts num_right_hand_key_points_with_non_zero_prob, right_hand_bounding_box
 * and right_hand_key_points_with_prob from Datum into OpenPoseHuman msg
 * @param input OpenPose op::Datum
 * @param output human message
 * @param index of the human inside Datum
 */
void getRightHandKeyPoints(const op::Datum &datum,  openpose_ros_msgs::OpenPoseHuman &human, int humanID);

OpenPoseROSIO::OpenPoseROSIO(OpenPose &openPose): it_(nh_)
{
    // Subscribe to input video feed and publish human lists as output
    std::string image_topic;
    std::string output_topic;
    std::string output_topic_2;
    std::string output_topic_video;

    nh_.param("/openpose_ros_node/image_topic", image_topic, std::string("/camera/image_raw"));
    nh_.param("/openpose_ros_node/output_topic", output_topic, std::string("/openpose_ros/human_list"));
    nh_.param("/openpose_ros_node/output_topic_2", output_topic_2, std::string("/openpose_ros/human_count"));
    nh_.param("/openpose_ros_node/output_topic_video", output_topic_video, std::string("/openpose_ros/image"));

    image_sub_ = it_.subscribe(image_topic, 1, &OpenPoseROSIO::processImage, this);
    openpose_human_list_pub_ = nh_.advertise<openpose_ros_msgs::OpenPoseHumanList>(output_topic, 10);
    openpose_human_count_pub_ = nh_.advertise<std_msgs::String>(output_topic_2, 10);
    openpose_image_ = nh_.advertise<sensor_msgs::Image>(output_topic_video, 15);

    cv_img_ptr_ = nullptr;
    openpose_ = &openPose;
}

void OpenPoseROSIO::processImage(const sensor_msgs::ImageConstPtr& msg)
{
    convertImage(msg);
    std::shared_ptr<std::vector<op::Datum>> datumToProcess = createDatum();

    bool successfullyEmplaced = openpose_->waitAndEmplace(datumToProcess);
    
    // Pop frame
    std::shared_ptr<std::vector<op::Datum>> datumProcessed;
    if (successfullyEmplaced && openpose_->waitAndPop(datumProcessed))
    {
        publishImageTopics(datumProcessed);
        publishPersonCountTopic(datumProcessed);
        publishHumanListTopic(datumProcessed);
    }
    else
    {
        op::log("Processed datum could not be emplaced.", op::Priority::High,
                __LINE__, __FUNCTION__, __FILE__);
    }
}

void OpenPoseROSIO::convertImage(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv_img_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        rgb_image_header_ = msg->header;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

std::shared_ptr<std::vector<op::Datum>> OpenPoseROSIO::createDatum()
{
    // Close program when empty frame
    if (cv_img_ptr_ == nullptr)
    {
        return nullptr;
    }
    else // if (cv_img_ptr_ == nullptr)
    {
        // Create new datum
        auto datumsPtr = std::make_shared<std::vector<op::Datum>>();
        datumsPtr->emplace_back();
        auto& datum = datumsPtr->at(0);

        // Fill datum
        datum.cvInputData = cv_img_ptr_->image;

        return datumsPtr;
    }
}

bool OpenPoseROSIO::display(const std::shared_ptr<std::vector<op::Datum>>& datumsPtr)
{
    // User's displaying/saving/other processing here
        // datum.cvOutputData: rendered frame with pose or heatmaps
        // datum.poseKeypoints: Array<float> with the estimated pose
    char key = ' ';
    if (datumsPtr != nullptr && !datumsPtr->empty())
    {
        cv::imshow("User worker GUI", datumsPtr->at(0).cvOutputData);
        // Display image and sleeps at least 1 ms (it usually sleeps ~5-10 msec to display the image)
        key = (char)cv::waitKey(1);
    }
    else
        op::log("Nullptr or empty datumsPtr found.", op::Priority::High, __LINE__, __FUNCTION__, __FILE__);
    return (key == 27);
}

void OpenPoseROSIO::publishImageTopics(const std::shared_ptr<std::vector<op::Datum>>& datumsPtr){
    auto outputIm = datumsPtr->at(0).cvOutputData;
    cv::putText(outputIm, "Person count: " + std::to_string(datumsPtr->at(0).poseKeypoints.getSize(0)),
                cv::Point(25,25),
                cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cv::Scalar(255,255,255),1);


    sensor_msgs::ImagePtr img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", outputIm).toImageMsg();
    openpose_image_.publish(img);
}

void OpenPoseROSIO::publishPersonCountTopic(const std::shared_ptr<std::vector<op::Datum>>& datumsPtr)
{
    std_msgs::String msg;
    msg.data = "Person Count: " + std::to_string(datumsPtr->at(0).poseKeypoints.getSize(0));

    openpose_human_count_pub_.publish(msg);
}

void OpenPoseROSIO::publishHumanListTopic(const std::shared_ptr<std::vector<op::Datum>>& datumsPtr)
{
    openpose_ros_msgs::OpenPoseHumanList msg;
    const auto &datum = datumsPtr->at(0);
    msg.num_humans = datum.poseKeypoints.getSize(0);
    msg.human_list.resize(msg.num_humans);

    for (auto i = 0; i < msg.num_humans; ++i) {
        auto &human = msg.human_list[i];
        getBodyKeyPoints(datum, human, i);
        getFaceKeyPoints(datum, human, i);
        getLeftHandKeyPoints(datum, human, i);
        getRightHandKeyPoints(datum, human, i);
    }

    openpose_human_list_pub_.publish(msg);
}


void getBodyKeyPoints(const op::Datum &datum,  openpose_ros_msgs::OpenPoseHuman &human, int humanID)
{
    assert (humanID < datum.poseKeypoints.getSize(0));

    int countNonZeroProb = 0;
    for (int i = 0; i < NUM_BODY_PARTS; ++i) {
        auto &pointWithProb = human.body_key_points_with_prob[i];
        pointWithProb.x    = datum.poseKeypoints[{humanID, i, 0}];
        pointWithProb.y    = datum.poseKeypoints[{humanID, i, 1}]; //humanID * HUMAN_DATA_SIZE + 3*i + 1];
        pointWithProb.prob = datum.poseKeypoints[{humanID, i, 2}];
        if (pointWithProb.prob != 0)
            ++countNonZeroProb;
    }
    human.num_body_key_points_with_non_zero_prob = countNonZeroProb;
}

void getFaceKeyPoints(const op::Datum &datum,  openpose_ros_msgs::OpenPoseHuman &human, int humanID)
{
    if (humanID >= datum.faceKeypoints.getSize(0))
        return;

    int countNonZeroProb = 0;
    for (auto i = 0; i < NUM_FACE_PARTS; ++i) {
        auto &pointWithProb = human.face_key_points_with_prob[i];
        pointWithProb.x    = datum.faceKeypoints[{humanID, i, 0}];
        pointWithProb.y    = datum.faceKeypoints[{humanID, i, 1}];
        pointWithProb.prob = datum.faceKeypoints[{humanID, i, 2}]; //humanID * FACE_DATA_SIZE + 3*i + 2];
        if (pointWithProb.prob != 0)
            ++countNonZeroProb;
    }
    human.num_face_key_points_with_non_zero_prob = countNonZeroProb;

    auto &rect = datum.faceRectangles[humanID];
    human.face_bounding_box.height = rect.height;
    human.face_bounding_box.width  = rect.width;
    human.face_bounding_box.x      = rect.x;
    human.face_bounding_box.y      = rect.y;
}

void getLeftHandKeyPoints(const op::Datum &datum,  openpose_ros_msgs::OpenPoseHuman &human, int humanID)
{
    if (humanID >= datum.handKeypoints[int(Hand::left)].getSize(0))
        return;

    const auto &leftHand = datum.handKeypoints[int(Hand::left)];
    int countNonZeroProb = 0;
    for (auto i = 0; i < NUM_HAND_PARTS; ++i) {
        auto &pointWithProb = human.left_hand_key_points_with_prob[i];
        pointWithProb.x    = leftHand[{humanID, i, 0}];
        pointWithProb.y    = leftHand[{humanID, i, 1}];
        pointWithProb.prob = leftHand[{humanID, i, 2}];
        if (pointWithProb.prob != 0)
            ++countNonZeroProb;
    }
    human.num_left_hand_key_points_with_non_zero_prob = countNonZeroProb;

    auto &rect = datum.handRectangles[humanID][int(Hand::left)];
    human.left_hand_bounding_box.height = rect.height;
    human.left_hand_bounding_box.width  = rect.width;
    human.left_hand_bounding_box.x      = rect.x;
    human.left_hand_bounding_box.y      = rect.y;
}

void getRightHandKeyPoints(const op::Datum &datum,  openpose_ros_msgs::OpenPoseHuman &human, int humanID)
{
    if (humanID >= datum.handKeypoints[int(Hand::right)].getSize(0))
        return;

    const auto &rightHand = datum.handKeypoints[int(Hand::right)];
    int countNonZeroProb = 0;
    for (auto i = 0; i < NUM_HAND_PARTS; ++i) {
        auto &pointWithProb = human.right_hand_key_points_with_prob[i];
        pointWithProb.x    = rightHand[{humanID, i, 0}];
        pointWithProb.y    = rightHand[{humanID, i, 1}];
        pointWithProb.prob = rightHand[{humanID, i, 2}];
        if (pointWithProb.prob != 0)
            ++countNonZeroProb;
    }
    human.num_right_hand_key_points_with_non_zero_prob = countNonZeroProb;

    auto &rect = datum.handRectangles[humanID][int(Hand::right)];
    human.right_hand_bounding_box.height = rect.height;
    human.right_hand_bounding_box.width  = rect.width;
    human.right_hand_bounding_box.x      = rect.x;
    human.right_hand_bounding_box.y      = rect.y;
}

