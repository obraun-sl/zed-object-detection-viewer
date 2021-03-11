///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2017, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

/*********************************************************************************
 ** This sample demonstrates how to capture 3D point cloud and detected objects  **
 **      with the ZED SDK and display the result in an OpenGL window. 	        **
 **********************************************************************************/

// ZED includes
#include <sl/Camera.hpp>

// Sample includes
#include "GLViewer.hpp"

// Using std and sl namespaces
using namespace std;
using namespace sl;

Camera zed;
bool exit_=false;
bool newFrame=false;

// Flag to enable/disable the tracklet merger module.
// Tracklet merger allows to reconstruct trajectories from objects from object detection module by using Re-ID between objects.
// For example, if an object is not seen during some time, it can be re-ID to a previous ID if the matching score is high enough
#define TRACKLET_MERGER 1


void parseArgs(int argc, char **argv,sl::InitParameters& param)
{
    if (argc > 1 && string(argv[1]).find(".svo")!=string::npos) {
        // SVO input mode
        param.input.setFromSVOFile(argv[1]);
        cout << "[Sample] Using SVO File input: " << argv[1] << endl;
    } else if (argc > 1 && string(argv[1]).find(".svo")==string::npos) {
        string arg = string(argv[1]);
        unsigned int a,b,c,d,port;
        if (sscanf(arg.c_str(),"%u.%u.%u.%u:%d", &a, &b, &c, &d,&port) == 5) {
            // Stream input mode - IP + port
            string ip_adress = to_string(a)+"."+to_string(b)+"."+to_string(c)+"."+to_string(d);
            param.input.setFromStream(sl::String(ip_adress.c_str()),port);
            cout<<"[Sample] Using Stream input, IP : "<<ip_adress<<", port : "<<port<<endl;
        }
        else  if (sscanf(arg.c_str(),"%u.%u.%u.%u", &a, &b, &c, &d) == 4) {
            // Stream input mode - IP only
            param.input.setFromStream(sl::String(argv[1]));
            cout<<"[Sample] Using Stream input, IP : "<<argv[1]<<endl;
        }
        else if (arg.find("HD2K")!=string::npos) {
            param.camera_resolution = sl::RESOLUTION::HD2K;
            cout<<"[Sample] Using Camera in resolution HD2K"<<endl;
        } else if (arg.find("HD1080")!=string::npos) {
            param.camera_resolution = sl::RESOLUTION::HD1080;
            cout<<"[Sample] Using Camera in resolution HD1080"<<endl;
        } else if (arg.find("HD720")!=string::npos) {
            param.camera_resolution = sl::RESOLUTION::HD720;
            cout<<"[Sample] Using Camera in resolution HD720"<<endl;
        } else if (arg.find("VGA")!=string::npos) {
            param.camera_resolution = sl::RESOLUTION::VGA;
            cout<<"[Sample] Using Camera in resolution VGA"<<endl;
        }
    } else {
        //
    }
}

#if TRACKLET_MERGER
std::deque<sl::Objects> objects_tracked_queue;
std::deque<sl::Timestamp> timestamp_queue;
std::map<unsigned long long,sl::Objects> objects_map;
std::map<unsigned long long,Pose> camPoseMap_ms;
std::map<unsigned long long,sl::Mat> image_map;
std::map<unsigned long long,sl::Mat> depth_map;



///
/// \brief ingestPoseInMap
/// \param ts: timestamp of the pose
/// \param pose : sl::Pose of the camera
/// \param batch_duration_sc: duration in seconds in order to remove past elements.
///
void ingestPoseInMap(sl::Timestamp ts, sl::Pose pose, int batch_duration_sc)
{
    std::map<unsigned long long,Pose>::iterator it = camPoseMap_ms.begin();
    for(auto it = camPoseMap_ms.begin(); it != camPoseMap_ms.end(); ) {
        if(it->first<ts.getMilliseconds() - (unsigned long long)batch_duration_sc*1000)
            it = camPoseMap_ms.erase(it);
        else
            ++it;
    }

    camPoseMap_ms[ts.getMilliseconds()]=pose;
}

void ingestImageInMap(sl::Timestamp ts, sl::Mat image, int batch_duration_sc)
{
    /*std::map<unsigned long long,sl::Mat>::iterator it = image_map.begin();
    for(auto it = image_map.begin(); it != image_map.end(); ) {
        if(it->first<ts.getMilliseconds() - (unsigned long long)batch_duration_sc*1000)
            it = image_map.erase(it);
        else
            ++it;
    }*/
    image_map[ts.getMilliseconds()].clone(image);
}

void ingestDepthInMap(sl::Timestamp ts, sl::Mat depth, int batch_duration_sc)
{
    /*std::map<unsigned long long,sl::Mat>::iterator it = depth_map.begin();
    for(auto it = depth_map.begin(); it != depth_map.end(); ) {
        if(it->first<ts.getMilliseconds() - (unsigned long long)batch_duration_sc*1000)
            it = depth_map.erase(it);
        else
            ++it;
    }*/

    depth_map[ts.getMilliseconds()].clone(depth);

}
///
/// \brief findClosestPoseFromTS : find closest sl::Pose according to timestamp. Use when resampling is used in tracklet merger, since generated objects can have a different
/// timestamp than the camera timestamp. If resampling==0, then std::map::find() will be enough.
/// \param timestamp in milliseconds. ( at least in the same unit than camPoseMap_ms)
/// \return sl::Pose found.
///
sl::Pose findClosestPoseFromTS(unsigned long long timestamp)
{
    sl::Pose pose = sl::Pose();
    unsigned long long ts_found = 0;
    if (camPoseMap_ms.find(timestamp)!=camPoseMap_ms.end()) {
        ts_found = timestamp;
        pose = camPoseMap_ms[timestamp];
    }
    else
    {
        std::map<unsigned long long,Pose>::iterator it = camPoseMap_ms.begin();
        unsigned long long diff_max_time = ULONG_LONG_MAX;
        while(it!=camPoseMap_ms.end())
        {
            long long diff = abs((long long)timestamp - (long long)it->first);
            if (diff<diff_max_time)
            {
                pose = it->second;
                diff_max_time = diff;
                ts_found = it->first;
            }
            it++;
        }
    }
    return pose;
}


sl::Mat findClosestImageFromTS(unsigned long long timestamp)
{
    sl::Mat image = sl::Mat();
    unsigned long long ts_found = 0;
    if (image_map.find(timestamp)!=image_map.end()) {
        ts_found = timestamp;
        image = image_map[timestamp];
    }
    else
    {
        std::map<unsigned long long,sl::Mat>::iterator it = image_map.begin();
        unsigned long long diff_max_time = ULONG_LONG_MAX;
        while(it!=image_map.end())
        {
            long long diff = abs((long long)timestamp - (long long)it->first);
            if (diff<diff_max_time)
            {
                image = it->second;
                diff_max_time = diff;
                ts_found = it->first;
            }
            it++;
        }
    }
    return image;
}

sl::Objects findClosestObjectsFromTS(unsigned long long timestamp)
{
    sl::Objects objs = sl::Objects();
    unsigned long long ts_found = 0;
    if (objects_map.find(timestamp)!=objects_map.end()) {
        ts_found = timestamp;
        objs = objects_map[timestamp];
    }
    else
    {
       /* std::map<unsigned long long,sl::Objects>::iterator it = objects_map.begin();
        unsigned long long diff_max_time = ULONG_LONG_MAX;
        while(it!=objects_map.end())
        {
            long long diff = abs((long long)timestamp - (long long)it->first);
            if (diff<diff_max_time)
            {
                objs = it->second;
                diff_max_time = diff;
                ts_found = it->first;
            }
            it++;
        }*/
    }
    return objs;
}

sl::Mat findClosestDepthFromTS(unsigned long long timestamp)
{
    sl::Mat image = sl::Mat();
    unsigned long long ts_found = 0;
    if (depth_map.find(timestamp)!=depth_map.end()) {
        ts_found = timestamp;
        image = depth_map[timestamp];
    }
    else
    {
        std::map<unsigned long long,sl::Mat>::iterator it = depth_map.begin();
        unsigned long long diff_max_time = ULONG_LONG_MAX;
        while(it!=depth_map.end())
        {
            long long diff = abs((long long)timestamp - (long long)it->first);
            if (diff<diff_max_time)
            {
                image = it->second;
                diff_max_time = diff;
                ts_found = it->first;
            }
            it++;
        }
    }
    return image;
}


///
/// \brief ingestInObjectsQueue : convert a list of trajectory from SDK retreiveBatchTrajectories to a sorted list of sl::Objects
/// \n Use this function to fill a std::deque<sl::Objects> that can be considered and used as a stream of objects with a delay.
/// \param trajs from retreiveBatchTrajectories
///
void ingestInObjectsQueue(std::vector<sl::Trajectory> trajs)
{
    // If list is empty, do nothing.
    if (trajs.empty())
        return;

    // add objects in map with timestamp as a key.
    // This ensure

    for (int i=0;i<trajs.size();i++)
    {
        sl::Trajectory current_traj = trajs.at(i);

        // Impossible but still better to check...
        if (current_traj.timestamp.size()!=current_traj.position.size())
            continue;


        //For each sample, construct a objetdata and put it in the corresponding sl::Objects
        for (int j=0;j<current_traj.timestamp.size();j++)
        {
            sl::Timestamp ts = current_traj.timestamp.at(j);
            sl::ObjectData newObjectData;
            newObjectData.id = current_traj.ID;
            newObjectData.tracking_state = current_traj.tracking_state;
            newObjectData.position = current_traj.position.at(j);
            newObjectData.label = current_traj.label;
            newObjectData.sublabel = current_traj.sublabel;
            newObjectData.bounding_box.clear();
            for (int k=0;k<current_traj.bounding_box.at(j).size();k++)
                newObjectData.bounding_box.push_back(current_traj.bounding_box.at(j).at(k));


            if (objects_map.find(ts.getMilliseconds())!=objects_map.end())
                objects_map[ts.getMilliseconds()].object_list.push_back(newObjectData);
            else
            {
                sl::Objects current_obj;
                current_obj.timestamp.setMilliseconds(ts.getMilliseconds());
                current_obj.is_new = true;
                current_obj.is_tracked = true;
                current_obj.object_list.push_back(newObjectData);
                objects_map[ts.getMilliseconds()] = current_obj;
            }
        }
    }

    return;
}
#endif

void run() {

    RuntimeParameters rtp;
    rtp.sensing_mode = SENSING_MODE::FILL;
    while(!exit_) {
        if (!newFrame) {
            if (zed.grab(rtp) == sl::ERROR_CODE::SUCCESS) {
                newFrame=true;
            }
        }
        sl::sleep_us(100);
    }
}

#include <opencv2/opencv.hpp>

int main(int argc, char **argv) {

 #if 0
    cv::Mat imge_no_track;
    cv::Mat imge_with_track;

    for (int i=1;i<680-90;i++)
    {
        char name_w_track[128];
        char name_n_track[128];
        char name_out_track[128];
        sprintf(name_w_track,"/home/obraun/Documents/obraun_github/build-zed-object-detection-viewer-Desktop-Debug/with_tracklet/test_%04d.png",i+90);
        sprintf(name_n_track,"/home/obraun/Documents/obraun_github/build-zed-object-detection-viewer-Desktop-Debug/no_tracklet/test_%04d.png",i);
        sprintf(name_out_track,"result_%04d.png",i);


        imge_with_track = cv::imread(std::string(name_w_track));
        imge_no_track = cv::imread(std::string(name_n_track));

        cv::putText(imge_with_track,"With",cv::Point(400,60),2,2.0,cv::Scalar(40,255,40),2.5);
        cv::putText(imge_no_track,"Without",cv::Point(400,60),2,2.0,cv::Scalar(40,40,255),2.5);


        cv::Mat result = cv::Mat(1080,3840,CV_8UC3,1);
        cv::Mat result_save;
        imge_no_track.copyTo(result(cv::Rect(0,0,1920,1080)));
        imge_with_track.copyTo(result(cv::Rect(1920,0,1920,1080)));

        cv::resize(result,result_save,cv::Size(1920,540));


        cv::imwrite(name_out_track,result_save);
        /*cv::imshow("no",imge_no_track);
        cv::imshow("yes",imge_with_track);

        cv::waitKey(5);*/


    }

    return 0;
#endif

    // Create ZED objects
    InitParameters initParameters;
    initParameters.depth_mode = sl::DEPTH_MODE::PERFORMANCE;
    initParameters.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
    initParameters.coordinate_units = sl::UNIT::METER;
    initParameters.sdk_verbose = true;
    initParameters.depth_maximum_distance =15.f; //For object detection, Objects after 15meters may not be precise enough.
    parseArgs(argc,argv, initParameters);

    // Open the camera
    std::cout<<" Opening Camera"<<std::endl;
    ERROR_CODE zed_error = zed.open(initParameters);
    if (zed_error != ERROR_CODE::SUCCESS) {
        std::cout << sl::toVerbose(zed_error) << "\nExit program." << std::endl;
        zed.close();
        return 1; // Quit if an error occurred
    }

    Resolution resolution = zed.getCameraInformation().camera_resolution;
    auto camera_parameters = zed.getCameraInformation(resolution).calibration_parameters.left_cam;

    //Only ZED2 has object detection
    if (zed.getCameraInformation().camera_model!=sl::MODEL::ZED2) {
        std::cout<<" ERROR : Use ZED2 Camera only"<<std::endl;
        exit(0);
    }

    // Enable Position tracking (mandatory for object detection)
    std::cout<<" Enable Positional Tracking "<<std::endl;
    sl::PositionalTrackingParameters positional_tracking_parameters;
    positional_tracking_parameters.set_as_static = true;
    zed_error =  zed.enablePositionalTracking(positional_tracking_parameters);
    if (zed_error != ERROR_CODE::SUCCESS) {
        std::cout << sl::toVerbose(zed_error) << "\nExit program." << std::endl;
        zed.close();
        return 1;
    }

    // Enable the Objects detection module
    std::cout<<" Enable Object Detection Module"<<std::endl;
    sl::ObjectDetectionParameters obj_det_params;
    obj_det_params.image_sync = true;
    zed_error = zed.enableObjectDetection(obj_det_params);
    if (zed_error != ERROR_CODE::SUCCESS) {
        std::cout << sl::toVerbose(zed_error) << "\nExit program." << std::endl;
        zed.close();
        return 1;
    }

    // Create OpenGL Viewer
    GLViewer viewer;
    viewer.init(argc, argv, camera_parameters);

    // Object Detection runtime parameters
    float object_confidence = 50.f;
    ObjectDetectionRuntimeParameters objectTracker_parameters_rt;
    objectTracker_parameters_rt.detection_confidence_threshold = object_confidence;
    objectTracker_parameters_rt.object_class_filter.clear();
    objectTracker_parameters_rt.object_class_filter.push_back(sl::OBJECT_CLASS::PERSON);


#if TRACKLET_MERGER
    std::cout<<" Enable Tracklet merger Module"<<std::endl;
    sl::BatchTrajectoryParameters trajectory_parameters;
    trajectory_parameters.resampling_rate = 0;
    trajectory_parameters.batch_duration = 2.f;
    zed_error = zed.enableBatchTrajectories(trajectory_parameters);
    if (zed_error != ERROR_CODE::SUCCESS) {
        std::cout << sl::toVerbose(zed_error) << "\nExit program." << std::endl;
        zed.close();
        return EXIT_FAILURE;
    }
#endif


    // Create ZED Objects
    Objects objects;

    sl::Timestamp current_im_ts;
    sl::Mat pDepth,pImage;

    sl::Mat b_image;
    sl::Mat b_depth;

    // Capture Thread (grab will run in the thread)
    exit_=false;
    //std::thread runner(run);
    sl::Timestamp init_app_ts = 0ULL;
    sl::Timestamp init_queue_ts = 0ULL;
    // Update 3D loop
    std::cout<<" Start Viewing loop"<<std::endl;
    while (viewer.isAvailable()) {
        RuntimeParameters rtp;
        rtp.sensing_mode = SENSING_MODE::FILL;
        if (zed.grab(rtp) == sl::ERROR_CODE::SUCCESS) {
#if TRACKLET_MERGER

            zed.retrieveObjects(objects, objectTracker_parameters_rt);
            zed.retrieveMeasure(pDepth, MEASURE::DEPTH, MEM::CPU);
            zed.retrieveImage(pImage, VIEW::LEFT, MEM::CPU);
            current_im_ts = zed.getTimestamp(sl::TIME_REFERENCE::IMAGE);
            timestamp_queue.push_back(current_im_ts);
            ingestImageInMap(current_im_ts,pImage,40000.0);
            ingestDepthInMap(current_im_ts,pDepth,40000.0);
            if (init_app_ts.data_ns==0ULL)
                init_app_ts =  zed.getTimestamp(sl::TIME_REFERENCE::IMAGE);

            std::vector<sl::Trajectory> trajectories;
            zed.retrieveBatchTrajectories(trajectories);
            ingestInObjectsQueue(trajectories);
            if (objects_map.size()>0 && image_map.size()>0)
            {
                sl::Timestamp new_ts = timestamp_queue.front();
                sl::Mat tmp_image=findClosestImageFromTS(new_ts.getMilliseconds());
                tmp_image.copyTo(b_image);
                tmp_image.free();
                image_map[new_ts.getMilliseconds()].free();
                image_map.erase(new_ts.getMilliseconds());

                sl::Mat tmp_depth=findClosestDepthFromTS(new_ts.getMilliseconds());
                tmp_depth.copyTo(b_depth);
                tmp_depth.free();
                depth_map[new_ts.getMilliseconds()].free();
                depth_map.erase(new_ts.getMilliseconds());

                sl::Objects tracked_merged_obj = findClosestObjectsFromTS(new_ts.getMilliseconds());
                //Update GL view
                b_image.updateGPUfromCPU();
                b_depth.updateGPUfromCPU();
                std::cout<<" tracked_merged_obj : "<<tracked_merged_obj.object_list.size()<<std::endl;
                viewer.updateData(b_image,b_depth, tracked_merged_obj,tracked_merged_obj.timestamp);
                timestamp_queue.pop_front();
                viewer.render();
            }
            else
            {
                //std::cout<<" - No Image - "<<std::endl;
                //std::cout<<" SIZES : "<<objects_map.size()<<" // "<<image_map.size()<<std::endl;
            }

            newFrame=false;

#else
            //Retrieve Images and Z-Buffer
            zed.retrieveMeasure(pDepth, MEASURE::DEPTH, MEM::GPU);
            zed.retrieveImage(pImage, VIEW::LEFT, MEM::GPU);
            //Retrieve Objects
            zed.retrieveObjects(objects, objectTracker_parameters_rt);
            current_im_ts = zed.getTimestamp(sl::TIME_REFERENCE::IMAGE);
            newFrame=false;
            //Update GL view
            viewer.updateData(pImage,pDepth, objects,current_im_ts);

#endif


        }
        else
            sleep_ms(1);
    }

    // OUT
    exit_=true;
    //runner.join();
    objects.object_list.clear();

    // Disable modules
    zed.disablePositionalTracking();
    zed.disableObjectDetection();
    zed.close();
    return 0;
}
