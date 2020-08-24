
#include <iostream>
#include <string>
#include "depthai/device.hpp"
#include <fstream>
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "oak-d_publisher/depth_map_publisher.hpp"

using namespace std;
    
DepthMapPublisher::DepthMapPublisher(
            const std::string& config_file_path,
            const std::string& depth_map_topic,
            const std::string& landmark_topic){

        


}

int main(){

    ros::init(argc, argv, "oak_publisher");
    ros::NodeHandle n;

    std::ifstream myFile("../config.json");
    std::ostringstream tmp;
    tmp << myFile.rdbuf();
    std::string config = tmp.str();
// ref: https://github.com/luxonis/depthai/blob/f26f8c6a5008ae32a388d482e0f16c72d428decf/depthai.py#L199

    Device d("", true);
    // cout << "------------found---------" << endl;
    cout << config << endl;
    std::shared_ptr<CNNHostPipeline> pipeline = d.create_pipeline(config);
    ros::Rate loop_rate(20);
    std::tuple<
        std::list<std::shared_ptr<NNetPacket>>,
        std::list<std::shared_ptr<HostDataPacket>>>

    pipeline.getAvailableNNetAndDataPackets(false);
    while(ros::ok()){
        
        
        
    



    }

   



    int wait;
    cin >> wait;

    return 0;
}


int main(){
    using namespace std;
    cout << "Hello World!" << endl;

    // std::string config = "{\
    //     \"streams\": [{\"name\":\"depth_raw\", \"max_fps\": 30.0},],\
    // \"depth\":\
    // {\
    //     \"calibration_file\": consts.resource_paths.calib_fpath,\
    //     \"padding_factor\": 0.3,\
    //     \"depth_limit_m\": 10.0,\
    //     \"confidence_threshold\" : 0.5,\
    // },\
    // }";

    // std::string config = "{\
    //     \"streams\": [{\"name\":\"depth_raw\", \"max_fps\": 30.0},],\
    // \"depth\":\
    // {\
    //     \"padding_factor\": 0.3,\
    //     \"depth_limit_m\": 10.0,\
    //     \"confidence_threshold\" : 0.5,\
    // },\
    // }";

std::string config = "{streams: [left, right],}";

    std::ifstream myFile("../config.json");
    std::ostringstream tmp;
    tmp << myFile.rdbuf();
    config = tmp.str();


    Device d("", true);
    cout << "------------found---------" << endl;
    cout << config << endl;
    std::shared_ptr<CNNHostPipeline> pipeline = d.create_pipeline(config);

    std::tuple<
        std::list<std::shared_ptr<NNetPacket>>,
        std::list<std::shared_ptr<HostDataPacket>>> packets; 
        int i = 0;
while(true){
        packets = pipeline->getAvailableNNetAndDataPackets(true);
        
        for(auto sub_packet :  get<1>(packets)){
                cout << sub_packet->stream_name <<"::-------::" << sub_packet->size() <<endl;
        }
    i++;
    if(i == 10)
    break;
}
    int wait;
    cin >> wait;

    return 0;
}