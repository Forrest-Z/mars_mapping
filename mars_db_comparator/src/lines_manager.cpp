#include "mars_db_comparator/lines_manager.h"

LinesManager::LinesManager(ros::NodeHandle& nh_):nh(nh_)
{
    nh.param<std::string>("lines_file_path", lines_file_path, "");

    loadLinesFile(lines_file_path);
}

void LinesManager::loadLinesFile(const std::string& file_path)
{
    std::ifstream file(file_path);

    if(!file.is_open())
    {
        ROS_ERROR_STREAM("Fail to open file !!");
        return;
    }else{
        ROS_INFO_STREAM("Open "<< file_path);
    }

    getSstream(file) >> submap_num;
    submap_lines_array.clear();
    for(unsigned int i = 0;i < submap_num;i++)
    {
        hough_line_msgs::Lines lines;
        int lines_num;

        getSstream(file) >> lines_num;
        lines.lines.resize(lines_num);
        
        for(unsigned int j = 0;j < lines_num;j++)
        {
            getSstream(file) >> lines.lines[j].angle >> lines.lines[j].rho >> lines.lines[j].score;
        }
        
        submap_lines_array.push_back(lines);
    }

    // for(int i = 0;i < submap_lines_array.size();i++)
    // {
    //     for(int j = 0;j < submap_lines_array[i].lines.size();j++)
    //     {
    //         ROS_INFO_STREAM(submap_lines_array[i].lines[j].angle << " " <<
    //                         submap_lines_array[i].lines[j].rho << " " <<
    //                         submap_lines_array[i].lines[j].score);
            
    //     }
    // }

    file.close();
    ROS_INFO_STREAM("Success to read lines from file !!");
}

std::stringstream LinesManager::getSstream(std::ifstream& file)
{
    std::string inputLine; 
    std::stringstream ss;

    getline(file,inputLine);
    ss.str(inputLine);

    return ss;
}
