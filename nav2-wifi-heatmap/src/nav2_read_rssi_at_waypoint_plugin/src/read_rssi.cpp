#include <fstream>
#include <string>
#include <iostream>
// Function for reading rssi
int read_rssi(){
    std::fstream rssi_file;
    rssi_file.open("/net_expose/wireless",std::ios::in);
    if (rssi_file.is_open()){
        std::string labels;
        std::string values;
        std::string line;
        for(int i = 0;i <= 4;i++){
            switch(i){
                case 1:
                    std::getline(rssi_file,labels);
                    break;
                case 2:
                    std::getline(rssi_file,values);
                    break;
                default:
                    std::getline(rssi_file,line);
                    break;
            }
        }
        // Find where 'level' occures
        std::cout << values << std::endl;
        std::cout << labels << std::endl;
        if (values.empty()){
            std::cout << "Skipping measurement" << std::endl;
            return 1;
        }
        std::size_t level_idx = labels.find("level",0);
        int rssi = std::stoi(values.substr(level_idx,4));
        std::cout << rssi << std::endl; 
    return rssi;
    } 
    std::cout << "rssi not read" << std::endl;
    return 1; 
}