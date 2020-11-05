#include "mars_slam/map/map_manager.h"

MapManager::MapManager(std::shared_ptr<ros::NodeHandle>& nh_,std::shared_ptr<PoseOptimizer>& pose_optimizer_):
nh(nh_),
pose_optimizer(pose_optimizer_),
stop_pub_global_map(false)
{
    // multiple threads speed up
    nh->param<int>("omp_max_thread_num", omp_max_thread_num, 4);

    nh->param<int>("initial_map_size", initial_map_size, 100);
    nh->param<double>("map_resolution", map_resolution, 0.05);
    nh->param<double>("change_submap_dis", change_submap_dis, 2);
    nh->param<double>("map_publsih_frequency", map_publsih_frequency, 1);
    nh->param<bool>("pub_submap", pub_submap, false);

    // init viewer
    map_info_viewer = std::make_unique<MapInfoViewer>(*nh);

    // init map stacks for loop closure detection
    map_stacks = std::make_shared<MapStacks>(*nh);

    // create new submap
    addNewSubmap(tf::Pose::getIdentity());
    map_info_viewer->updateSubmapCenters(submaps);
    second_submap = submaps[0];

    // create new global map
    global_map = std::make_shared<OccMap>(initial_map_size,map_resolution,tf::Pose::getIdentity(),-1);
    middle_map = std::make_shared<OccMap>(initial_map_size,map_resolution,tf::Pose::getIdentity(),-1);                                  
    global_map->initMapPub(*nh);

    // open threads
    map_pub_thread = std::async(std::launch::async, &MapManager::publishMapLoop,this);
    ROS_INFO_STREAM("Open map thread, start publish map !!");
    ROS_INFO_STREAM(" ");

    
}

void MapManager::publishMapLoop()
{
    omp_set_num_threads(omp_max_thread_num); 
    ros::Rate r(map_publsih_frequency); 

    while(ros::ok())
    {
        publishMap();

        ROS_DEBUG_STREAM("Pub map !!");
        r.sleep();
    }
}

void MapManager::dealWithNewScanData(const PCLScanCloudPtr& scan_cloud,
                                     const tf::Pose& pose,
                                     int& current_submap_serial_num,
                                     int& scan_serial_num)
{  
    if(checkChangeSubmap(pose))
    {
        newSubmapProcess(pose);
    }

    if(submaps.size()>1){
        first_submap->addNewScanData(scan_cloud,pose);
        second_submap->addNewScanData(scan_cloud,pose);
    }else{
        second_submap->addNewScanData(scan_cloud,pose);
    }

    current_submap_serial_num = submaps.size() - 1;
    scan_serial_num = second_submap->getCurrentScanNum() - 1;

    pose_optimizer->addNewVertex(current_submap_serial_num,pose);
}

bool MapManager::checkChangeSubmap(const tf::Pose& pose)
{
    if(dist(pose,second_submap->getCenter()) > change_submap_dis)
        return true;
    else
        return false;
}

double MapManager::dist(const tf::Pose& a,const tf::Pose& b)
{
    double dx = a.getOrigin().x() - b.getOrigin().x();
    double dy = a.getOrigin().y() - b.getOrigin().y();
    return sqrt(pow(dx,2) + pow(dy,2));
}

void MapManager::deleteMap()
{
    middle_map_mutex.lock();
    middle_map->deleteMap();
    middle_map_mutex.unlock();
    global_map->deleteMap();

    for(int i = 0;i<submaps.size();i++)
    {
        submaps[i]->deleteMap();
    }
}

void MapManager::publishMap()
{
    if(pub_submap)
        for(unsigned int i = 0;i < submaps.size();i++)
            submaps[i]->publishMap();

    if(!stop_pub_global_map)
        publishGlobalMap();

    pose_optimizer->pubConstraintMarker();
}

void MapManager::publishGlobalMap()
{
    middle_map_mutex.lock();
    global_map->copyMap(*middle_map);
    middle_map_mutex.unlock();
    if(submaps.size()%2 == 0)
        global_map->combineMap(*first_submap);
    else
        global_map->combineMap(*second_submap);
    global_map->publishMap();
}

std::shared_ptr<OccMap>& MapManager::getScanMatchingMap()
{
    if(submaps.size() > 1)
        return first_submap;
    else
        return second_submap;
}

void MapManager::addNewSubmap(const tf::Pose& pose)
{
    int serial_num = submaps.size();

    // create new submap
    std::shared_ptr<OccMap> new_submap = std::make_shared<OccMap>
                                      (initial_map_size,map_resolution,pose,serial_num);
    if(pub_submap)
        new_submap->initMapPub(*nh);
    submaps.push_back(new_submap);
}

void MapManager::newSubmapProcess(const tf::Pose& pose)
{
    submaps.back()->setOwnScanNum();
    if(submaps.size()%2 == 0){
        middle_map_mutex.lock();
        middle_map->combineMap(*(submaps[submaps.size()-2]));
        middle_map_mutex.unlock();
    }
    addNewSubmap(pose);
    map_info_viewer->updateSubmapCenters(submaps);
    int map_num = submaps.size();
    first_submap = submaps[map_num - 2];
    first_submap->copyOriginalMapToHalfMap();
    first_submap->createTransformedMap();
    second_submap = submaps[map_num - 1];

    // add new stack
    if(submaps.size() > 2)
        map_stacks->updateMapStack(*(submaps[submaps.size()-3]));
}

void MapManager::updateSubmapByOptimizationPoses(const int& last_submap_serial_num)
{
    ROS_INFO_STREAM("Start updating map");
    ros::Time t1 = ros::Time::now();
    #pragma omp parallel for
    for(unsigned int i = 0;i <= last_submap_serial_num;i++)
    {
        submaps[i]->updateMapByOptimizationPoses(pose_optimizer->getVertices(i),
                                                 pose_optimizer->getVertices(i+1));
        map_stacks->setTransformedCenter(i,submaps[i]->getCenter());
    }
    map_info_viewer->updateSubmapCenters(submaps);

    middle_map_mutex.lock();
    middle_map->initMap(initial_map_size,map_resolution,tf::Pose::getIdentity(),-1,false);
    for(unsigned int i = 0;i < floor((submaps.size()-1)/2)*2;i++)
        middle_map->combineMapByTransformedMap(*(submaps[i]));
    // middle_map->copyMap(*(submaps[0]));

    // for(unsigned int i = 2;i < submaps.size()-2;i=i+2)
    //     middle_map->combineMap(*(submaps[i]));

    middle_map_mutex.unlock();

    ros::Time t2 = ros::Time::now();
    ROS_INFO_STREAM("Finish update all map: " << t2-t1 << "s");
}

bool MapManager::callSaveMap(const std::string& file_name)
{
    // remove useless info
    for(unsigned int i = 0;i < submaps.size();i++)
        submaps[i]->narrowMap();
    global_map->narrowMap();

    ROS_INFO_STREAM("Finish narrow maps, map numbers: " << submaps.size());
    
    if(saveMap(file_name))
        return true;
    else
        return false;
}

bool MapManager::saveMap(const std::string& file_path)
{
    std::ofstream file;
    std::string file_name = file_path + ".db";
    file.open(file_name);
    if(!file.is_open())
    {
        ROS_ERROR_STREAM("Fail to open file !!");
        return false;
    }else{
        ROS_INFO_STREAM("Open "<< file_name);
    }

    saveMapInfo(file);

    file.close();
    return true;
}

void MapManager::saveMapInfo(std::ofstream &file)
{
    file << submaps.size() << "\n";
    file << map_resolution << "\n";

    // write submap data
    for(int i = 0;i < submaps.size();i++)
    {
        submaps[i]->saveMap(file);
    }

    global_map->saveMap(file);
}