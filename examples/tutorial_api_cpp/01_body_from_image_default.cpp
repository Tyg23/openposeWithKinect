// ----------------------------- OpenPose C++ API Tutorial - Example 1 - Body from image -----------------------------
// It reads an image, process it, and displays it with the pose keypoints.

// Third-party dependencies
#include <opencv2/opencv.hpp>
// Command-line user interface
#define OPENPOSE_FLAGS_DISABLE_POSE
#include <openpose/flags.hpp>
// OpenPose dependencies
#include <openpose/headers.hpp>
#include <k4a/k4a.h>
#include <k4a/k4a.hpp>

// Custom OpenPose flags
// Producer
DEFINE_string(image_path, "/home/y/Downloads/20210913/Train/Train/images/img1.jpg",
    "Process an image. Read all standard formats (jpg, png, bmp, etc.).");
// Display
DEFINE_bool(no_display,                 false,
    "Enable to disable the visual display.");


std::vector<k4a_float2_t> pixels;
// This worker will just read and return all the jpg files in a directory
void display(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr)
{
    try
    {
        // User's displaying/saving/other processing here
            // datum.cvOutputData: rendered frame with pose or heatmaps
            // datum.poseKeypoints: Array<float> with the estimated pose
        if (datumsPtr != nullptr && !datumsPtr->empty())
        {
            // Display image
            const cv::Mat cvMat = OP_OP2CVCONSTMAT(datumsPtr->at(0)->cvOutputData);
            if (!cvMat.empty())
            {
                cv::imshow(OPEN_POSE_NAME_AND_VERSION + " - Tutorial C++ API", cvMat);
                cv::imwrite("dectedPic.jpg",cvMat);
                cv::waitKey(0);
            }
            else
                op::opLog("Empty cv::Mat as output.", op::Priority::High, __LINE__, __FUNCTION__, __FILE__);
        }
        else
            op::opLog("Nullptr or empty datumsPtr found.", op::Priority::High);
    }
    catch (const std::exception& e)
    {
        op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
    }
}

void printKeypoints(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr)
{
    try
    {
        // Example: How to use the pose keypoints
        if (datumsPtr != nullptr && !datumsPtr->empty())
        {
            // //Alternative 1
            // op::opLog("Body keypoints: " + datumsPtr->at(0)->poseKeypoints.toString(), op::Priority::High);

            // // Alternative 2
            // op::opLog(datumsPtr->at(0)->poseKeypoints, op::Priority::High);

            // // Alternative 3
            // std::cout << datumsPtr->at(0)->poseKeypoints << std::endl;

            // Alternative 4 - Accesing each element of the keypoints
            op::opLog("\nKeypoints:", op::Priority::High);
            const auto& poseKeypoints = datumsPtr->at(0)->poseKeypoints;
            op::opLog("Person pose keypoints:", op::Priority::High);
            for (auto person = 0 ; person < poseKeypoints.getSize(0) ; person++)
            {
                op::opLog("Person " + std::to_string(person) + " (x, y, score):", op::Priority::High);
                for (auto bodyPart = 0 ; bodyPart < poseKeypoints.getSize(1) ; bodyPart++)
                {
                    std::string valueToPrint;
                    for (auto xyscore = 0 ; xyscore < poseKeypoints.getSize(2) ; xyscore++)
                        valueToPrint += std::to_string(   poseKeypoints[{person, bodyPart, xyscore}]   ) + " ";                    
                    op::opLog(valueToPrint, op::Priority::High);
                }
            }
            for (int i = 2; i < 5; i++)
            {
                k4a_float2_t p;
                p.xy.x = poseKeypoints[{0, i, 0}];
                p.xy.y = poseKeypoints[{0, i, 1}];
                pixels.push_back(p);
            }
            
            op::opLog(" ", op::Priority::High);
        }
        else
            op::opLog("Nullptr or empty datumsPtr found.", op::Priority::High);
    }
    catch (const std::exception& e)
    {
        op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
    }
}

int tutorialApiCpp(const cv::Mat cvImageToProcess)
{
    try
    {
        op::opLog("Starting OpenPose demo...", op::Priority::High);
        const auto opTimer = op::getTimerInit();

        // Configuring OpenPose
        op::opLog("Configuring OpenPose...", op::Priority::High);
        op::Wrapper opWrapper{op::ThreadManagerMode::Asynchronous};
        // Set to single-thread (for sequential processing and/or debugging and/or reducing latency)
        if (FLAGS_disable_multi_thread)
            opWrapper.disableMultiThreading();

        // Starting OpenPose
        op::opLog("Starting thread(s)...", op::Priority::High);
        opWrapper.start();

        // Process and display image
        // const cv::Mat cvImageToProcess = cv::imread(FLAGS_image_path);
        const op::Matrix imageToProcess = OP_CV2OPCONSTMAT(cvImageToProcess);
        auto datumProcessed = opWrapper.emplaceAndPop(imageToProcess);
        if (datumProcessed != nullptr)
        {
            printKeypoints(datumProcessed);
            if (!FLAGS_no_display)
                display(datumProcessed);
        }
        else
            op::opLog("Image could not be processed.", op::Priority::High);

        // Measuring total time
        op::printTime(opTimer, "OpenPose demo successfully finished. Total time: ", " seconds.", op::Priority::High);

        // Return
        return 0;
    }
    catch (const std::exception&)
    {
        return -1;
    }
}

float getSlope(const k4a_float2_t p1,const k4a_float2_t p2)
{
    float k;
    k=(p1.xy.y-p2.xy.y)/(p1.xy.x-p2.xy.x);
    return k;
}

k4a_float2_t getP(float k,const k4a_float2_t p,float distance,int flag)
{
    k4a_float2_t pp;
    pp.xy.x=p.xy.x+distance*flag;
    pp.xy.y=k*(pp.xy.x-p.xy.x)+p.xy.y;
    return pp;
}

int main(int argc, char *argv[])
{
    // Parsing command line flags
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    //找到并打开设备
    uint32_t count = k4a_device_get_installed_count();
    if (count == 0)
    {
        std::cout << "No k4a devices attached!\n";
    }

    k4a_device_t device=NULL;
    // Open the first plugged in Kinect device 打开设备
    if (K4A_FAILED(k4a_device_open(K4A_DEVICE_DEFAULT, &device)))
    {
        std::cout << "Failed to open k4a device!\n";
    }

    // Get the size of the serial number 获取设备序列号
    size_t serial_size = 0;
    k4a_device_get_serialnum(device, NULL, &serial_size);

    // Allocate memory for the serial, then acquire it
    char *serial = (char *)(malloc(serial_size));
    k4a_device_get_serialnum(device, serial, &serial_size);
    std::cout << "Opened device: %s\n";
    std::cout << serial << std::endl;
    free(serial);

    // Configure a stream of 720P BRGA color data at 30 frames per second 设置参数
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.color_resolution = K4A_COLOR_RESOLUTION_720P;
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    config.synchronized_images_only = true;

    k4a_calibration_t calibration;
    if (K4A_RESULT_SUCCEEDED !=
        k4a_device_get_calibration(device, config.depth_mode, config.color_resolution, &calibration))
    {
        std::cout << "Failed to get calibration\n";
    }
    k4a_transformation_t transformation = k4a_transformation_create(&calibration);

    // Start the camera with the given configuration
    if (K4A_FAILED(k4a_device_start_cameras(device, &config)))
    {
        std::cout << "Failed to start cameras!\n";
        k4a_device_close(device);
    }

    k4a_capture_t capture=NULL;
	const int32_t TIMEOUT_IN_MS = 10000;
    switch (k4a_device_get_capture(device, &capture, TIMEOUT_IN_MS))
	{
	case K4A_WAIT_RESULT_SUCCEEDED:
		break;
	case K4A_WAIT_RESULT_TIMEOUT:
		std::cout << "Timed out waiting for a capture\n";
		break;
	case K4A_WAIT_RESULT_FAILED:
		std::cout << "Failed to read a capture\n";
		break;
	}    

    k4a_image_t color_image=NULL;
    k4a_image_t depth_image=NULL;
    k4a_image_t color_depth_image=NULL;

    color_image=k4a_capture_get_color_image(capture);
    depth_image=k4a_capture_get_depth_image(capture);
    int color_image_width_pixels = k4a_image_get_width_pixels(color_image);
    int color_image_height_pixels = k4a_image_get_height_pixels(color_image);

    k4a_image_t point_cloud_image = NULL;
	if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
												 color_image_width_pixels,
												 color_image_height_pixels,
												 color_image_width_pixels * 3 * (int)sizeof(int16_t),
												 &point_cloud_image))
	{
		std::cout << "Failed to create point cloud image\n";
	}

    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
                                                 color_image_width_pixels,
                                                 color_image_height_pixels,
                                                 color_image_width_pixels * (int)sizeof(uint16_t),
                                                 &color_depth_image))
    {
        printf("Failed to create transformed depth image\n");
    }

    if (color_image == 0)
    {
        printf("Failed to get color image from capture\n");
    }
	if (depth_image == 0)
	{
		std::cout << "Failed to get depth image from capture\n";
	}


    if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_color_camera(transformation, depth_image, color_depth_image))
    {
        printf("Failed to compute transformed depth image\n");
    }
    cv::Mat cv_rgbImage_with_alpha=cv::Mat(color_image_height_pixels,color_image_width_pixels,CV_8UC4,
                        (void*)k4a_image_get_buffer(color_image),cv::Mat::AUTO_STEP);
    cv::Mat cvImageToProcess;
    cv::cvtColor(cv_rgbImage_with_alpha, cvImageToProcess, cv::COLOR_BGRA2BGR);
    cv::imwrite("pic.jpg",cvImageToProcess);


    // cv::Mat cv_rgbImage_wi=cv::Mat(color_image_height_pixels,color_image_width_pixels,CV_16UC1,
    //                     (void*)k4a_image_get_buffer(color_depth_image),cv::Mat::AUTO_STEP);
    // cv::imshow("1",cv_rgbImage_wi);cv::waitKey(0);
    
    if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_point_cloud(transformation,
                                                                              color_depth_image,
                                                                              K4A_CALIBRATION_TYPE_COLOR,
                                                                              point_cloud_image))
    {
        printf("Failed to compute point cloud\n");
        return false;
    }

    struct color_point_t
	{
		int16_t xyz[3];
		uint8_t rgb[3];
	};

	std::vector<color_point_t> points;

	//int width = k4a_image_get_width_pixels(point_cloud_image);
	//int height = k4a_image_get_height_pixels(transformed_color_image);

	int16_t *point_cloud_image_data = (int16_t *)(void *)k4a_image_get_buffer(point_cloud_image);
	uint8_t *color_image_data = k4a_image_get_buffer(color_image);

	for (int i = 0; i < color_image_width_pixels * color_image_height_pixels; i++)
	{
		color_point_t point;
		point.xyz[0] = point_cloud_image_data[3 * i + 0];
		point.xyz[1] = point_cloud_image_data[3 * i + 1];
		point.xyz[2] = point_cloud_image_data[3 * i + 2];
		if (point.xyz[2] == 0)
		{
			continue;
		}

		point.rgb[0] = color_image_data[4 * i + 0];
		point.rgb[1] = color_image_data[4 * i + 1];
		point.rgb[2] = color_image_data[4 * i + 2];
		uint8_t alpha = color_image_data[4 * i + 3];

		if (point.rgb[0] == 0 && point.rgb[1] == 0 && point.rgb[2] == 0 && alpha == 0)
		{
			continue;
		}

		points.push_back(point);
	}
    
    std::ofstream infile1("xyz.txt");
	assert(infile1.is_open());

    for (int i = 0; i < points.size(); i++)
    {
        // int a = points[i].xyz[0];
        // int b = points[i].xyz[1];
        // int c = points[i].xyz[2];
        // bool xx = a > -400 && a < 400;
        // bool yy = b > -300 && b < 300;
        // bool zz = c > 200 && c < 700;

        // if (xx && yy && zz)
        // {
        infile1 << points[i].xyz[0];
        infile1 << " ";
        infile1 << points[i].xyz[1];
        infile1 << " ";
        infile1 << points[i].xyz[2];
        // infile << " ";
        // infile << (float)points[i].rgb[2];
        // infile << " ";
        // infile << (float)points[i].rgb[1];
        // infile << " ";
        // infile << (float)points[i].rgb[0];
        infile1 << std::endl;
        // }
    }

    std::ofstream infile2("xyzrgb.txt");
	assert(infile2.is_open());

    for (int i = 0; i < points.size(); i++)
    {
        // int a = points[i].xyz[0];
        // int b = points[i].xyz[1];
        // int c = points[i].xyz[2];
        // bool xx = a > -400 && a < 400;
        // bool yy = b > -300 && b < 300;
        // bool zz = c > 200 && c < 700;

        // if (xx && yy && zz)
        // {
        infile2 << points[i].xyz[0];
        infile2 << " ";
        infile2 << points[i].xyz[1];
        infile2 << " ";
        infile2 << points[i].xyz[2];
        infile2 << " ";
        infile2 << (float)points[i].rgb[2];
        infile2 << " ";
        infile2 << (float)points[i].rgb[1];
        infile2 << " ";
        infile2 << (float)points[i].rgb[0];
        infile2 << std::endl;
        // }
    }

    
    // cv::imshow("1",cvImageToProcess);cv::waitKey(0);
    tutorialApiCpp(cvImageToProcess);
    k4a_float3_t ppp;
    int valid;
    int16_t *depth_image_data = (int16_t *)(void *)k4a_image_get_buffer(color_depth_image);

    float k1=-1/getSlope(pixels[0],pixels[1]);
    float k2=-1/getSlope(pixels[1],pixels[2]);
    std::vector<k4a_float2_t> clipPixels;
    for (size_t i = 0; i < 8; i++)
    {
        clipPixels.push_back(getP(k1, pixels[0], i, -1));
        clipPixels.push_back(getP(k1, pixels[0], i, 1));
    }
    for (size_t i = 0; i < 8; i++)
    {
        clipPixels.push_back(getP(k1, pixels[1], i, -1));
        clipPixels.push_back(getP(k1, pixels[1], i, 1));
    }
    for (size_t i = 0; i < 8; i++)
    {
        clipPixels.push_back(getP(k2, pixels[1], i, -1));
        clipPixels.push_back(getP(k2, pixels[1], i, 1));
    }
    for (size_t i = 0; i < 8; i++)
    {
        clipPixels.push_back(getP(k2, pixels[2], i, -1));
        clipPixels.push_back(getP(k2, pixels[2], i, 1));
    }
    

    std::ofstream infile3("featureXYZ.txt");
    assert(infile3.is_open());
    for (size_t i = 0; i < 3; i++)
    {
        float depth = depth_image_data[color_image_width_pixels * (int)pixels[i].xy.y + (int)pixels[i].xy.x];
        k4a_calibration_2d_to_3d(&calibration, &pixels[i], depth, K4A_CALIBRATION_TYPE_COLOR, K4A_CALIBRATION_TYPE_COLOR, &ppp, &valid);
        std::cout << pixels[i].xy.x << "###" << pixels[i].xy.y << std::endl;
        std::cout << ppp.xyz.x << "&&&" << ppp.xyz.y << "&&&" << ppp.xyz.z << std::endl;
        infile3 << ppp.xyz.x << " " << ppp.xyz.y << " " << ppp.xyz.z << std::endl;
    }

    std::ofstream infile4("clipXYZ.txt");
    assert(infile4.is_open());
    for (size_t i = 0; i < clipPixels.size(); i++)
    {
        float depth = depth_image_data[color_image_width_pixels * (int)clipPixels[i].xy.y + (int)clipPixels[i].xy.x];
        k4a_calibration_2d_to_3d(&calibration, &clipPixels[i], depth, K4A_CALIBRATION_TYPE_COLOR, K4A_CALIBRATION_TYPE_COLOR, &ppp, &valid);
        infile4 << ppp.xyz.x << " " << ppp.xyz.y << " " << ppp.xyz.z << std::endl;
    }

    // Running tutorialApiCpp
    // return tutorialApiCpp();
    return 0;
}
