/**
* Slamtec Robot STCM Parser Demo
*
* Created By Jason @ 2023-11-8
* Copyright (c) 2023 Shanghai SlamTec Co., Ltd.
*/
#include <iostream>
#include <rpos/core/detail/metadata_internal.h>
#include <rpos/robot_platforms/objects/composite_map_reader.h>
#include <rpos/robot_platforms/objects/grid_map_layer.h>
#include <rpos/robot_platforms/objects/line_map_layer.h>
#include <rpos/robot_platforms/objects/pose_map_layer.h>
#include <rpos/robot_platforms/objects/rectangle_area_map_layer.h>
#include <rpos/robot_platforms/objects/points_map_layer.h>

using namespace rpos::robot_platforms;

std::string filename = "";

void showHelp(std::string appName)
{
    std::cout << "SLAMWARE STCM Parser demo." << std::endl << \
        "Usage: \t [-f filename]  read stcm" << std::endl;
}

bool parseCommandLine(int argc, const char * argv[])
{
    bool opt_show_help = true;

    for (int pos = 1; pos < argc; ++pos)
    {
        const char* current = argv[pos];
        if (strcmp(current, "-h") == 0) {
            opt_show_help = true;
        }
        else if (strcmp(current, "-f") == 0) {
            if (++pos < argc) {
                filename = argv[pos];
                opt_show_help = false;
            }
        }
    }

    if (opt_show_help)
    {
        showHelp("stcm parser");
        return false;
    }

    return true;
}

int main(int argc, const char * argv[])
{
    if(!parseCommandLine(argc, argv) )
    {
        return 1;
    }


    try {
        objects::CompositeMapReader reader;
        auto compositeMap = reader.loadFile(filename);
        if (compositeMap->isMultiFloorMap())
        {
            std::cout << "This is multi floor map" << std::endl;
            std::string defaultFloorStr = "";
            if (compositeMap->metadata().tryGet(RPOS_COMPOSITEMAP_METADATA_KEY_DEFAULT, defaultFloorStr))
            {
                std::cout << "Default floor order: " << defaultFloorStr <<std::endl;
            }
        }

        for (auto mapLayer : compositeMap->maps())
        {
            std::cout <<"MapLayer:" << std::endl;
            std::cout << "    Type: " << mapLayer->getType() << std::endl;
            std::cout << "    Usage: " << mapLayer->getUsage() << std::endl;
            auto& metadata = mapLayer->metadata();
            std::string building;
            std::string floor; 
            std::string order;
            if (metadata.tryGet(RPOS_COMPOSITEMAP_METADATA_KEY_BUILDING, building) 
                && metadata.tryGet(RPOS_COMPOSITEMAP_METADATA_KEY_FLOOR, floor)
                && metadata.tryGet(RPOS_COMPOSITEMAP_METADATA_KEY_ORDER, order))
            {
                std::cout << "    Floor Identifier: " << building << "/" << floor <<", order: " << order << std::endl;
            }
            if (mapLayer->getType() == rpos::robot_platforms::objects::GridMapLayer::Type)
            {
                auto gridMap = boost::dynamic_pointer_cast<rpos::robot_platforms::objects::GridMapLayer>(mapLayer);
                if (gridMap)
                {
                    std::cout << "    Size: " << gridMap->mapData().size() << std::endl;
                    std::cout << "    Resolution: " << gridMap->getResolution().x() << std::endl;
                    std::cout << "    Dimension: " << gridMap->getDimension().x() << ", " << gridMap->getDimension().y() << std::endl;
                    std::cout << "    Location: " << gridMap->getOrigin().x() << ", " << gridMap->getOrigin().y() << std::endl;
                }
                rpos::core::Vector3f transform;
                if (metadata.tryGet<rpos::core::Vector3f>(RPOS_COMPOSITEMAP_METADATA_KEY_TRANSFORM, transform))
                {
                    std::cout << "    Transform from default floor: " << transform.x() << ", " << transform.y() << ", " << transform.z() << std::endl;
                }
            }
            else if (mapLayer->getType() == rpos::robot_platforms::objects::LineMapLayer::Type)
            {
                auto lineMap = boost::dynamic_pointer_cast<rpos::robot_platforms::objects::LineMapLayer>(mapLayer);
                if (lineMap)
                {
                    std::cout << "    Size: " << lineMap->lines().size()<< std::endl;
                }
            }
            else if (mapLayer->getType() == rpos::robot_platforms::objects::PoseMapLayer::Type)
            {
                auto poseMap = boost::dynamic_pointer_cast<rpos::robot_platforms::objects::PoseMapLayer>(mapLayer);
                if (poseMap)
                {
                    std::cout << "    Size: " << poseMap->poses().size() << std::endl;
                }
            }
            else if (mapLayer->getType() == rpos::robot_platforms::objects::RectangleAreaMapLayer::Type)
            {
                auto rectMap = boost::dynamic_pointer_cast<rpos::robot_platforms::objects::RectangleAreaMapLayer>(mapLayer);
                if (rectMap)
                {
                    std::cout << "    Size: " << rectMap->areas().size() << std::endl;
                }
            }
            else if (mapLayer->getType() == rpos::robot_platforms::objects::PointsMapLayer::Type)
            {
                auto pointsMap = boost::dynamic_pointer_cast<rpos::robot_platforms::objects::PointsMapLayer>(mapLayer);
                if (pointsMap)
                {
                    std::cout << "    Size: " << pointsMap->points().size()<< std::endl;
                }
            }
        }
    }
    catch (rpos::system::detail::ExceptionBase& e) { 
        std::cout << e.what()<<std::endl;
    }

    return 0;
}