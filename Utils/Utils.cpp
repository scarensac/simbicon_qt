#include "utils.h"

#include <sstream>
#include <iostream>
#include <algorithm>
#include <cctype>
#include "Globals.h"

/**
 * Output the message to a file...
 */
void logPrint(const char *format, ...){
    static char message[1024];
    va_list vl;

    va_start(vl, format);
    vsprintf(message, format, vl);
    va_end(vl);

    //in case the folde rdon't exist yet
    static bool first_time = true;
    if (first_time){
        system("mkdir out");
        first_time = false;
    }

    //TODO change this
    static FILE *fp = fopen("out\\log.txt", "wt");

    if (fp != NULL){
        fprintf(fp, "%s", message);
        fflush(fp);
    }
}


void debugLog(std::string msg){
    //TODO change this
    static FILE *fp = fopen("out\\debugLog.txt", "a");

    fprintf(fp, "%s", msg.c_str());
    fflush(fp);
}



/**
those method are helper to split a string
*/
std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    //split the string
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }

    if (!elems.empty()){
        //remove the endl from the last element
        std::stringstream ss2(elems.back());
        elems.pop_back();
        while (std::getline(ss2, item, '\n')) {
            elems.push_back(item);
        }
    }
    return elems;
}


std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}


/**
this function interpret the path (using the global configuration data path
*/
std::string interpret_path(std::string path){
    std::vector<std::string> splited_path;
    std::ostringstream oss;
    bool config_folder_reached;
    bool first_passage;

    splited_path = split(path, '/');
    //now we look for the configuration folder in the vector and we create the real path
    config_folder_reached = false;
    for (int i = 0; i < (int)splited_path.size(); ++i){
        //we ignore anything before the data folder
        std::string without_blank=splited_path[i];
        rmv_white_spaces(without_blank);
        if (!config_folder_reached){
            if ( without_blank == "configuration_data"){
                config_folder_reached = true;
                oss << Globals::data_folder_path;
                first_passage = true;
            }
            continue;
        }

        //so if we reach here we have only the things following the data folder
        if (first_passage){
            first_passage = false;
        }
        else{
            oss << "/";
        }
        oss << splited_path[i];
    }
    return oss.str();
}


#include <sys/stat.h>
std::string get_folder_path(std::string name, int lookup_nbr, std::string delim, std::string start_folder){
    std::stringstream oss;

    struct stat st;
    for (int i=0;i<lookup_nbr+1;++i){
        //now the tactic will be to look at every folder and check if we find a data folder
        std::stringstream oss2;
        oss2 << start_folder;
        oss2 << oss.str();
        oss2 << name;
        if (stat(oss2.str().c_str(), &st) == 0 && st.st_mode == 16895) {
            oss << name << delim;
            std::string path=oss.str();
            return path;
        }

        if (stat("src", &st) == 0 && st.st_mode == 16895){
            std::cout << "the configuration_data folder canno't be found" << std::endl;
            system("pause");
            exit(69);
        }

        oss << ".."<<delim;
    }
    std::cout << "the configuration_data folder cannot be found" << std::endl;
    system("pause");
    exit(69);

}


void rmv_white_spaces(std::string &s){
    s.erase(std::remove_if(s.begin(), s.end(), std::isspace), s.end());
}


std::string tab_string(int nb_tabulation)
{
    std::ostringstream oss;
    for (int i=0;i<nb_tabulation;++i){
        oss<<"  ";
    }
    return oss.str();
}

#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <chrono>
#include <random>

double generate_random_0_1()
{
    //*
    static bool init_done=false;
    if (!init_done){
        // initialize random seed:
        srand (time(NULL));
        init_done=true;
    }
    return (rand() % 1000)/1000.0;
    //*/
    /*
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    static std::default_random_engine generator (seed);
    static std::normal_distribution<double> distribution (0.5,0.2);
    double number;
    do{
        number=distribution(generator);
    }while(!((number>=0.0)&&(number<1.0)));
    return number;
    //*/
}

double radian_distance_signed(double from, double to){
    double dist=0;
    //first let's normalize in the interval 0  2PI for an easier handling
    if((from<0)||(from>(2*PI))){
        from=fmod(from,2*PI);
    }
    if((to<0)||(to>(2*PI))){
        to=fmod(to,2*PI);
    }

    //now a simple sustractio can handle all cases
    dist=to-from;

    //then take the shortest distance
    if(fabs(dist)>PI){
        dist=-1*(dist/fabs(dist))*(2*PI-fabs(dist));
    }

    return dist;

}
