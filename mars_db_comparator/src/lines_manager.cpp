#include "mars_db_comparator/lines_manager.h"

LinesManager::LinesManager(ros::NodeHandle& nh_):nh(nh_)
{
    nh.param<std::string>("lines_file_path", lines_file_path, "");

    nh.param<double>("parallel_threshold", parallel_threshold, 2.0);
    nh.param<double>("sigma_theta", sigma_theta, 1.0);
    nh.param<double>("sigma_dist", sigma_dist, 0.2);
    nh.param<int>("rank_size",rank_size,5);

    rank_num_array.resize(rank_size);
    rank_score_array.resize(rank_size);

    for(auto x:rank_score_array)
        x = 0;

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
    submap_lines_relations_array.clear();
    for(unsigned int i = 0;i < submap_num;i++)
    {
        hough_line_msgs::Lines lines;
        LinesRelationArray lines_relation_array;
        int lines_num;

        getSstream(file) >> lines_num;
        lines.lines.resize(lines_num);
        
        for(unsigned int j = 0;j < lines_num;j++)
        {
            getSstream(file) >> lines.lines[j].angle >> lines.lines[j].rho >> lines.lines[j].score;
        }

        getLineRelation(lines,lines_relation_array);
        
        submap_lines_array.push_back(lines);
        submap_lines_relations_array.push_back(lines_relation_array);
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

void LinesManager::getLineRelation(const hough_line_msgs::Lines& lines,LinesRelationArray& lines_relation_array)
{
    lines_relation_array.clear();

    // ROS_INFO_STREAM(" ");
    double totol_weight = 0;

    for(unsigned int i = 0;i < lines.lines.size();i++)
    {
        for(unsigned int j = i+1;j < lines.lines.size();j++)
        {
            hough_line_msgs::Line a = lines.lines[i];
            hough_line_msgs::Line b = lines.lines[j];

            double angle,dist;
            double w;

            angle = fabs(a.angle - b.angle);
            if(angle > 90)
            {
                angle = 180 - angle;
                dist = fabs(a.rho + b.rho);
            }else
            {
                dist = fabs(a.rho - b.rho);
            }
            w = sqrt(a.score*b.score);

            LinesRelation lr;
            lr.dist = dist;
            lr.intersection_angle = angle;
            lr.weight = w;
            totol_weight += w;

            // ROS_INFO_STREAM(dist << " "<<angle <<" "<<w);

            lines_relation_array.push_back(lr);
        }
    }

    if(totol_weight != 0)
        for(auto& x:lines_relation_array)
            x.weight/=totol_weight;
}

std::vector<int>& LinesManager::compareLinesRelationsWithDatabase(const LinesRelationArray& scan_lines_relation)
{
    // std::vector<int> totoal_score_array;
    // double max_score = 0;
    int submap_num;
    for(auto& x:rank_score_array)
        x = 0;

    for(int i = 0;i < submap_lines_relations_array.size();i++)
    {
        double total_score = 0;
        LinesRelationArray submap_lines_relation = submap_lines_relations_array[i];
    
        for(auto a:submap_lines_relation)
        {
            for(auto b:scan_lines_relation)
            {
                double score = sqrt(a.weight*b.weight)*angleScore(a,b);
                               
                if(checkParallel(a,b)) 
                    score*=distScore(a,b);

                total_score+=score;        
            }
        }
 
        // if(total_score > max_score)
        // {
        //     max_score = total_score;
        //     submap_num = i;
        // }

        if(total_score > rank_score_array[0])
        {
            rank_score_array[0] = total_score;
            rank_num_array[0] = i;

            for(int k = 0;k < rank_size-1;k++)
            {
                if(rank_score_array[k] > rank_score_array[k+1])
                {
                    double t = rank_score_array[k+1];
                    int t_ = rank_num_array[k+1];
                    rank_score_array[k+1] = rank_score_array[k];
                    rank_num_array[k+1] = rank_num_array[k];
                    rank_score_array[k] =t;
                    rank_num_array[k] = t_;
                }else
                {
                    break;
                }
                
            }
        }
        
    }

    for(int j = 0;j < rank_size;j++)
    {
        ROS_INFO_STREAM(rank_num_array[j] << " " << rank_score_array[j]);
    }

    return rank_num_array;
}

double LinesManager::angleScore(const LinesRelation& a,const LinesRelation& b)
{
    return exp(-pow(a.intersection_angle-b.intersection_angle,2)/(2*pow(sigma_theta,2)));
}

double LinesManager::distScore(const LinesRelation& a,const LinesRelation& b)
{
    return exp(-pow(a.dist-b.dist,2)/(2*pow(sigma_dist,2)));
}

bool LinesManager::checkParallel(const LinesRelation& a,const LinesRelation& b)
{
    return a.intersection_angle <= parallel_threshold && b.intersection_angle <= parallel_threshold;
}