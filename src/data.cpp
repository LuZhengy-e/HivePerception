#include<sstream>
#include "data.h"

namespace HIVE_SLAM{

void getFile(const std::string& dirname, std::vector<std::string>& files){
    struct dirent* ent = nullptr;
    DIR *dir = opendir(dirname.c_str());

    if(dir != nullptr){
        while ((ent = readdir(dir)) != NULL){
            if (strcmp(ent->d_name, ".") != 0 && strcmp(ent->d_name, "..") != 0)
            {
                // std::cout << ent->d_name << " has been saved" << std::endl;
                files.push_back(ent->d_name);
            }
        }
        closedir(dir);
    }
}

Kitti::Kitti(const std::string& filename): mrootDir(filename)
{
    std::cout << "Start construct Kitti Odometry datasets" << std::endl;
    const std::string labels = mrootDir + "/labels";
    const std::string pcds = mrootDir + "/velodyne_pcd";
    const std::string images = mrootDir + "/image_0";
    const std::string times = mrootDir + "/times.txt";
    const std::string calib = mrootDir + "/calib.txt";
    const std::string poses = mrootDir + "/02.txt";

    // 1. read poses
    std::ifstream fin_pose;
    fin_pose.open(poses.c_str());
    std::string pose_i;
    std::vector<cv::Mat> coords;
    while(!fin_pose.eof()){
        getline(fin_pose, pose_i);
        // std::cout << "cur pose is: " << pose_i.size() << std::endl;
        if(pose_i.empty()){
            continue;
        }
        cv::Mat pos(4, 4, CV_64F);
        cv::setIdentity(pos);

        int i = 0;
        int j = 0;
        std::string coord = "";
        for(std::string::iterator it = pose_i.begin(); it != pose_i.end(); ++it){
            if(*it == ' '){
                char* ptr;
                pos.at<double>(i, j) = std::strtod(coord.c_str(), &ptr);
                j += 1;
                if(j == 4){
                    i += 1;
                    j = 0;
                }
                coord = "";
            }
            else{
                coord += *it;
            }
        }
        char* ptr;
        pos.at<double>(i, j) = std::strtod(coord.c_str(), &ptr);
        coords.push_back(pos.clone());

        // check
        // for(std::vector<cv::Mat>::iterator it = coords.begin(); it != coords.end(); ++it){
        //     std::cout << (*it).at<double>(3, 3) << std::endl;
        // }
    }
    fin_pose.close();

    // 2. read all calib param
    std::ifstream fin_calib;
    fin_calib.open(calib.c_str());
    std::string K_i;
    std::vector<cv::Mat> calibs;
    cv::Mat Tcl(4, 4, CV_64F);
    cv::setIdentity(Tcl);
    int line = 0;
    while(!fin_calib.eof()){
        getline(fin_calib, K_i);
        // std::cout << "cur K is: " << K_i << std::endl;
        if(K_i.empty()){
            continue;
        }
        cv::Mat tmp_K(3, 4, CV_64F);
        
        int i = 0;
        int j = 0;
        std::string coord = "";
        for(std::string::iterator it = K_i.begin(); it != K_i.end(); ++it){
            if(*it == ' '){
                char* ptr;
                tmp_K.at<double>(i, j) = std::strtod(coord.c_str(), &ptr);
                j += 1;
                if(j == 4){
                    i += 1;
                    j = 0;
                }
                coord = "";
            }
            else{
                coord += *it;
            }
        }
        char* ptr;
        tmp_K.at<double>(i, j) = std::strtod(coord.c_str(), &ptr);

        if(line < 4){
            cv::Mat K = tmp_K.colRange(0, 3);
            calibs.push_back(K.clone());
        }
        else{
            cv::Rect2d rect = cv::Rect2d(0, 0, 4, 3); // cv::rect, change roi data
            tmp_K.copyTo(Tcl(rect));
        }

        ++line;
    }
    fin_calib.close();
    // check
    // for(int i = 0; i < 4; ++i){
    //     for(int j = 0; j < 4; ++j){
    //         std::cout << Tcl.at<double>(i, j) << ",";
    //     }
    //     std::cout << std::endl;
    // }

    mtrans_handler = CoordTranform(calibs[0], Tcl, Tcl); // body coord == lidar coord

    // 3. read images and construct KeyFrame
    std::vector<std::string> image_names;
    getFile(images, image_names);
    std::sort(image_names.begin(), image_names.end());
    cv::Mat K0 = calibs[0];

    if(coords.size() != image_names.size()){
        std::cerr << "pose: " << coords.size() << ", images" << image_names.size() << std::endl;
    }

    // temp, should be i < coord.size()
    for(size_t i = 0; i < 60; ++i){
        // temp, reduce keyframe numbers
        if(i % 5 != 0){
            continue;
        }
        
        std::string name = image_names[i];
        std::string image_path = images + "/" + name;
        // std::cout << "cur image is: " << image_path << std::endl;
        cv::Mat img = cv::imread(image_path);
        if(img.data == nullptr){
            std::cerr << name << " does not exist.." << std::endl;
        }
        // cv::imshow("img", img);
        // cv::waitKey(0.25);
        cv::Mat pos = coords[i];
        KeyFrame* pKF = new KeyFrame(img, pos.inv(), K0);
        mmptraj[pKF->getId()] = pKF;
    }

    // check
    // std::cout << "total " << HIVE_SLAM::KeyFrameId << " keyFrames has been created" << std::endl; // Interesting, KeyFrameId has changed in KeyFrame
    // for(std::map<int, KeyFrame*>::iterator it = mmptraj.begin(); it != mmptraj.end(); ++it){
    //     std::cout << it->first << "," << it->second->getId() << std::endl;
    // }

}

void Kitti::read_results(const std::string& in_file){
    const std::string labels = mrootDir + in_file;
    std::vector<std::string> label_names;
    getFile(labels, label_names);
    std::sort(label_names.begin(), label_names.end());

    Json::Reader reader;

    // check
    if(mmptraj.size() != label_names.size()){
        std::cerr << "label and keyframe need to align" << std::endl;
    }
    std::map<unsigned long int, std::map<int, std::map<char, cv::Mat> > > markers; // <keyframe_id, <group_id, <pos_id, pos_2d> > >
    std::map<unsigned long int, std::map<int, std::map<char, cv::Mat> > > poles; // <keyframe_id, <group_id, <pos_id, pos_2d> > >

    for(size_t i = 0; i < mmptraj.size(); ++i){
        std::string json_name_cur = labels + label_names[i];
        KeyFrame* pKF_cur = mmptraj[i];

        unsigned long int KF_cur_id = pKF_cur->getId();

        // read result from json file
        std::ifstream fin_cur;
        fin_cur.open(json_name_cur.c_str());
        Json::Value res_cur;
        if(!reader.parse(fin_cur, res_cur)){
            std::cerr << "read json file " << json_name_cur << " failed:" << std::endl;
            return;
        }
        if(!res_cur.isObject()){
            std::cerr << "read json with problem" << std::endl;
        }
        // check
        // std::cout << *(res_cur["shapes"].begin()) << std::endl;

        Json::Value shapes_cur = res_cur["shapes"];
        
        for(Json::Value::iterator it = shapes_cur.begin(); it != shapes_cur.end(); ++it){
            // temp, it will delete when other element has construct
            if((*it)["label"].asString().find("marker") != std::string::npos){
                int group_id = (*it)["group_id"].asInt();
                if((*it)["shape_type"].asString() == "point"){
                    cv::Mat u = (cv::Mat_<int>(2, 1) << (*it)["points"][0][0].asInt(), (*it)["points"][0][1].asInt());
                    char pos_id = *((*it)["label"].asString().end() - 1);
                    // check
                    // std::cout << u.at<int>(0, 0) << "," << u.at<int>(1, 0) << std::endl;
                    // std::cout << "pos_id is: " << pos_id << std::endl;
                    markers[KF_cur_id][group_id][pos_id] = u.clone();
                }
            }
            else if((*it)["label"].asString().find("pole") != std::string::npos){
                int group_id = (*it)["group_id"].asInt();
                if((*it)["shape_type"].asString() == "point"){
                    cv::Mat u = (cv::Mat_<int>(2, 1) << (*it)["points"][0][0].asInt(), (*it)["points"][0][1].asInt());
                    char pos_id = *((*it)["label"].asString().end() - 1);
                    // check
                    // std::cout << u.at<int>(0, 0) << "," << u.at<int>(1, 0) << std::endl;
                    // std::cout << "pos_id is: " << pos_id << std::endl;
                    poles[KF_cur_id][group_id][pos_id] = u.clone();
                }
            }
        }
    }
    
    if(mmptraj.size() != markers.size() || mmptraj.size() != poles.size()){
        std::cerr << "number of traj is: " << mmptraj.size() << "number of markers kf is: " << markers.size() << std::endl;
        return;
    }
    std::map<std::string, std::map<int, std::map<char, PointElement*> > > group_3D;
    std::map<std::string, std::map<int, std::map<char, double> > > minError;
    for(size_t i = 0; i < mmptraj.size() - 1; ++i){
        std::cout << "-------------------------------------------------" << std::endl;
        std::cout << "This is KeyFrame " << i << std::endl;
        KeyFrame* pKF_cur = mmptraj[i];
        KeyFrame* pKF_next = mmptraj[i + 1];
        
        // statistic markers
        std::map<int, std::map<char, cv::Mat> > groups = markers[i];
        for(std::map<int, std::map<char, cv::Mat> >::iterator group_it = groups.begin(); group_it != groups.end(); ++group_it){
            int group_id = group_it->first;
            if(markers[pKF_next->getId()].count(group_id) == 0){
                continue;
            }
            for(std::map<char, cv::Mat>::iterator pos_it = group_it->second.begin(); pos_it != group_it->second.end(); ++pos_it){
                if(markers[pKF_next->getId()][group_id].count(pos_it->first) == 0){
                    continue;
                }
                cv::Mat pos_cur = pos_it->second;
                cv::Mat pos_next = markers[pKF_next->getId()][group_id][pos_it->first];
                cv::Mat pos4D = utils::Triangulation(pKF_cur, pKF_next, pos_cur, pos_next);

                // update best position, minimize projection error
                
                cv::Mat proj_cur = mtrans_handler.Tcp(pos4D, pKF_cur->getPos()).rowRange(0, 2);

                cv::Mat pos_cur_double = pos_cur.clone();
                pos_cur_double.convertTo(pos_cur_double, CV_64F);

                double err = cv::norm(pos_cur_double - proj_cur);
                // check
                // std::cout << "cur pose is: " << std::endl << pKF_cur->getPos() << std::endl; 
                // std::cout << "next pose is: " << std::endl << pKF_next->getPos() << std::endl;
                // std::cout << "Projection error is: " << cv::norm(err) << std::endl;
                // std::cout << "Minimal error is: " << minError["marker"][group_id][pos_it->first] << std::endl;

                if(group_3D["marker"].count(group_id) == 0 || group_3D["marker"][group_id].count(pos_it->first) == 0){
                    Marker* marker = new Marker(pos4D.rowRange(0, 3), pKF_cur, true);
                    marker->setID();
                    marker->setType("marker");
                    marker->setPosType(pos_it->first);
                    group_3D["marker"][group_id][pos_it->first] = marker;
                    minError["marker"][group_id][pos_it->first] = cv::norm(err);
                }
                else if(cv::norm(err) < minError["pole"][group_id][pos_it->first]){
                    group_3D["marker"][group_id][pos_it->first]->setWorldPos(pos4D.rowRange(0, 3));
                    minError["marker"][group_id][pos_it->first] = cv::norm(err);
                }
            }
        }

        // statistic poles
        groups = poles[i];
        for(std::map<int, std::map<char, cv::Mat> >::iterator group_it = groups.begin(); group_it != groups.end(); ++group_it){
            int group_id = group_it->first;
            if(poles[pKF_next->getId()].count(group_id) == 0){
                continue;
            }
            for(std::map<char, cv::Mat>::iterator pos_it = group_it->second.begin(); pos_it != group_it->second.end(); ++pos_it){
                if(poles[pKF_next->getId()][group_id].count(pos_it->first) == 0){
                    continue;
                }
                cv::Mat pos_cur = pos_it->second;
                cv::Mat pos_next = poles[pKF_next->getId()][group_id][pos_it->first];
                cv::Mat pos4D = utils::Triangulation(pKF_cur, pKF_next, pos_cur, pos_next);

                // update best position, minimize projection error
                
                cv::Mat proj_cur = mtrans_handler.Tcp(pos4D, pKF_cur->getPos()).rowRange(0, 2);

                cv::Mat pos_cur_double = pos_cur.clone();
                pos_cur_double.convertTo(pos_cur_double, CV_64F);

                double err = cv::norm(pos_cur_double - proj_cur);
                // check
                // std::cout << "cur pose is: " << std::endl << pKF_cur->getPos() << std::endl; 
                // std::cout << "next pose is: " << std::endl << pKF_next->getPos() << std::endl;
                // std::cout << "Projection error is: " << cv::norm(err) << std::endl;
                // std::cout << "Minimal error is: " << minError["pole"][group_id][pos_it->first] << std::endl;

                if(group_3D["pole"].count(group_id) == 0 || group_3D["pole"][group_id].count(pos_it->first) == 0){
                    Pole* pole = new Pole(pos4D.rowRange(0, 3), pKF_cur, false);
                    pole->setID();
                    pole->setType("pole");
                    pole->setPosType(pos_it->first);
                    group_3D["pole"][group_id][pos_it->first] = pole;
                    minError["pole"][group_id][pos_it->first] = cv::norm(err);
                }
                else if(cv::norm(err) < minError["pole"][group_id][pos_it->first]){
                    group_3D["pole"][group_id][pos_it->first]->setWorldPos(pos4D.rowRange(0, 3));
                    minError["pole"][group_id][pos_it->first] = cv::norm(err);
                }
            }
        }
    }

    // check
    // Json::FastWriter writer;
    // Json::Value group_3D_json;
    // for(std::map<int, std::map<char, cv::Mat> >::iterator it = group_3D["marker"].begin(); it != group_3D["marker"].end(); ++it){
    //     Json::Value pos_id_json;
    //     for(std::map<char, cv::Mat>::iterator sub_it = it->second.begin(); sub_it != it->second.end(); ++sub_it){
    //         Json::Value pos_json;
    //         for(size_t i = 0; i < 3; ++i){
    //             pos_json.append(sub_it->second.at<double>(i, 0));
    //         }
    //         std::string key(1, sub_it->first);
    //         pos_id_json[key] = pos_json;
    //     }
    //     group_3D_json[std::to_string(it->first)] = pos_id_json;
    // }
    // std::string jsonstr = group_3D_json.toStyledString();
    // // std::cout << jsonstr << std::endl;
    // std::ofstream fout;
    // fout.open("markers.json");
    // fout << jsonstr;
    // fout.close();

}

void Kitti::bin2pcd(const std::string& in_file, const std::string& out_file){
    // load point cloud
    std::fstream input(in_file.c_str(), std::ios::in | std::ios::binary);
    if(!input.good()){
        std::cerr << "Could not read file: " << in_file << std::endl;
        exit(EXIT_FAILURE);
    }
    input.seekg(0, std::ios::beg);

    pcl::PointCloud<pcl::PointXYZI>::Ptr points (new pcl::PointCloud<pcl::PointXYZI>);

    int i;
    for (i=0; input.good() && !input.eof(); i++) {
        pcl::PointXYZI point;
        input.read((char *) &point.x, 3*sizeof(float));
        input.read((char *) &point.intensity, sizeof(float));
        points->push_back(point);
    }
    input.close();

    std::cout << "Read KTTI point cloud with " << i << " points, writing to " << out_file << std::endl;
    pcl::PCDWriter writer;

    // Save DoN features
    writer.write< pcl::PointXYZI > (out_file, *points, false);
}
};