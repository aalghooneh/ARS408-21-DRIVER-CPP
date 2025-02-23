

/* this is the decoder for conti ARS408-21 */


#include "decode_radars/decoder.h"
#include <boost/program_options.hpp>



int main(int argc, char** argv){

    ros::init(argc, argv, "radar_decoder_node");

    std::string mode;
    int id=-1;

    boost::program_options::options_description desc("available options");

    desc.add_options()
        ("help", " --mode:cluster, or, objecst, --id: id of sensor")
        ("mode", boost::program_options::value<std::string>(&mode),"set the mode")
        ("id", boost::program_options::value<int>(&id), "set the id");


    boost::program_options::variables_map vm;
    
    
    boost::program_options::store(
        boost::program_options::parse_command_line(argc,argv,desc),vm
    );

    boost::program_options::notify(vm);

    if (vm.count("help")){
        std::cout<<desc<<std::endl;
    }

    if (mode.empty()){
        std::cout<<"mode is not set"<<std::endl;
        return -1;
    }

    if (id==-1){
        std::cout<<"set up the id please"<<std::endl;
        return -2;
    }

    std::shared_ptr<decoder> DECODER;


    if (mode=="cluster"){
        std::cout<<"the mode is set to: "<<mode
        <<" the id is set to: "<<id<<std::endl;
        DECODER=std::make_shared<decoder>(id, _cluster);
    }
    else if (mode=="object"){
        std::cout<<"the mode is set to: "<<mode
        <<" the id is set to: "<<id<<std::endl;
        DECODER=std::make_shared<decoder>(id, _object);
    }

    ros::spin();

    return 0;

}