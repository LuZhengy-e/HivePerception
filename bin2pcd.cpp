#include<dirent.h>
#include "data.h"

// void getFile(const std::string& dirname, std::vector<std::string>& files){
//     struct dirent* ent = nullptr;
//     DIR *dir = opendir(dirname.c_str());

//     if(dir != nullptr){
//         while ((ent = readdir(dir)) != NULL){
//             if (strcmp(ent->d_name, ".") != 0 && strcmp(ent->d_name, "..") != 0)
//             {
//                 //std::cout << ent->d_name << std::endl;
//                 files.push_back(ent->d_name);
//             }
//         }
//         closedir(dir);
//     }
// }

int main(int argc, char* argv[]){
    if(argc != 3){
        std::cerr << "Please input the bin dir and pcd dir" << std::endl;
    }

    const std::string input = argv[1];
    const std::string output = argv[2];

    std::vector<std::string> files;
    files.reserve(10000);

    HIVE_SLAM::getFile(input, files);
    std::cout << files.size() << std::endl;

    for(auto file: files){
        std::string input_file = input + "/" + file;
        std::string output_file = output + "/" + file.substr(0, file.size() - 4) + ".pcd";

        // std::cout << "cur file is: " << output_file << std::endl;
        
        HIVE_SLAM::Kitti::bin2pcd(input_file, output_file);
    }

    std::cout << "All bin file has processed to .pcd" << std::endl;
    
    return 0;
}