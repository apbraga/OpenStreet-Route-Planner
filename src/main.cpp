#include <optional>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <io2d.h>
#include "route_model.h"
#include "render.h"
#include "route_planner.h"

using namespace std::experimental;

static std::optional<std::vector<std::byte>> ReadFile(const std::string &path)
{   
    std::ifstream is{path, std::ios::binary | std::ios::ate};
    if( !is )
        return std::nullopt;
    
    auto size = is.tellg();
    std::vector<std::byte> contents(size);    
    
    is.seekg(0);
    is.read((char*)contents.data(), size);

    if( contents.empty() )
        return std::nullopt;
    return std::move(contents);
}

//function to get input within boundary and type restrictions
//target variable is passed as reference, then the function won't need to return anything.
void GetCoordinate(float &coordinate, std::string name){
    //flag to check if input was completed
    bool pending = true;
    //request user input for as long as a valid input is not given
    while(pending){
        //prompt the user to input the value
        std::cout << "\n Input a number for 0 to 100 for " << name << "    :";
        std::cin >> coordinate;
        //check if the input type is not valid, by checking if the cin failed, and if the input number is within the map limits 0>100
        if(std::cin.fail() || (coordinate < 0 || coordinate > 100)){
            // inform user about the error
            std::cout << "\n ERROR - input is invalid, try again. Input must by a float from 0 to 100. \n";
            // clear cin for the following input
            std::cin.clear();
            char buf[BUFSIZ];
            std::cin >> buf;
        }
        else{
            // if input is valid, stop loop.
            pending = false;
        }
    }
}


int main(int argc, const char **argv)
{    
    std::string osm_data_file = "";
    if( argc > 1 ) {
        for( int i = 1; i < argc; ++i )
            if( std::string_view{argv[i]} == "-f" && ++i < argc )
                osm_data_file = argv[i];
    }
    else {
        std::cout << "To specify a map file use the following format: " << std::endl;
        std::cout << "Usage: [executable] [-f filename.osm]" << std::endl;
        osm_data_file = "../map.osm";
    }
    
    std::vector<std::byte> osm_data;
 
    if( osm_data.empty() && !osm_data_file.empty() ) {
        std::cout << "Reading OpenStreetMap data from the following file: " <<  osm_data_file << std::endl;
        auto data = ReadFile(osm_data_file);
        if( !data )
            std::cout << "Failed to read." << std::endl;
        else
            osm_data = std::move(*data);
    }
    
    // TODO 1: Declare floats `start_x`, `start_y`, `end_x`, and `end_y` and get
    // user input for these values using std::cin. Pass the user input to the
    // RoutePlanner object below in place of 10, 10, 90, 90.

    //initialize variable with an invalid value (not within 0>100)
    float start_x=-1.0, start_y=-1.0, end_x=-1.0, end_y=-1.0;
    //prompt user for coordinate input until input is valid
    //function will use the given variable as reference
    GetCoordinate(start_x, "start_x");
    GetCoordinate(start_y, "start_y");
    GetCoordinate(end_x, "end_x");
    GetCoordinate(end_y, "end_y");
    
    // Build Model.
    RouteModel model{osm_data};

    // Create RoutePlanner object and perform A* search.
    RoutePlanner route_planner{model, start_x, start_y, end_x, end_y};
    route_planner.AStarSearch();

    std::cout << "Distance: " << route_planner.GetDistance() << " meters. \n";

    // Render results of search.
    Render render{model};

    auto display = io2d::output_surface{400, 400, io2d::format::argb32, io2d::scaling::none, io2d::refresh_style::fixed, 30};
    display.size_change_callback([](io2d::output_surface& surface){
        surface.dimensions(surface.display_dimensions());
    });
    display.draw_callback([&](io2d::output_surface& surface){
        render.Display(surface);
    });
    display.begin_show();
}
