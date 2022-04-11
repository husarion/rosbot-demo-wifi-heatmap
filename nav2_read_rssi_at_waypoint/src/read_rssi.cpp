#include <fstream>
#include <string>

int read_rssi(){
    std::fstream rssi_file;
    rssi_file.open("/fakenet_/wireless",std::ios::in);
    if (rssi_file.is_open()){
        std::string str;
        for(int i = 0;i < 2;i++){
            std::getline(rssi_file,str);
        }
        if(str.length() < 29){
            std::cout << "String not loaded, skipping measurement" << std::endl;
            return NULL;
        }
        std::getline(rssi_file,str);
        str = str.substr(29,3);
        rssi_file.close();
        return stoi(str);
    } 
    return 0;  
}