#include "map.h"

bool Map::get_map(const char* FileName)
{

    tinyxml2::XMLElement *root = 0, *map = 0, *element = 0, *mapnode;

    std::string value;
    std::stringstream stream;

    bool hasGridMem(false), hasGrid(false), hasHeight(false), hasWidth(false);

    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(FileName) != tinyxml2::XMLError::XML_SUCCESS)
    {
        std::cout << "Error opening XML file!" << std::endl;
        return false;
    }
    root = doc.FirstChildElement(CNS_TAG_ROOT);
    if (!root)
    {
        std::cout << "Error! No '" << CNS_TAG_ROOT << "' tag found in XML file!" << std::endl;
        return false;
    }
    map = root->FirstChildElement(CNS_TAG_MAP);
    if (!map)
    {
        std::cout << "Error! No '" << CNS_TAG_MAP << "' tag found in XML file!" << std::endl;
        return false;
    }

    for (mapnode = map->FirstChildElement(); mapnode; mapnode = mapnode->NextSiblingElement())
    {
        element = mapnode->ToElement();
        value = mapnode->Value();
        std::transform(value.begin(), value.end(), value.begin(), ::tolower);

        stream.str("");
        stream.clear();
        stream << element->GetText();

        if (!hasGridMem && hasHeight && hasWidth)
        {
            grid.resize(height);
            for (int i = 0; i < height; ++i)
                grid[i].resize(width);
            hasGridMem = true;
        }

        if (value == CNS_TAG_HEIGHT)
        {
            if (hasHeight)
            {
                std::cout << "Warning! Duplicate '" << CNS_TAG_HEIGHT << "' encountered." << std::endl;
                std::cout << "Only first value of '" << CNS_TAG_HEIGHT << "' =" << height << "will be used."
                          << std::endl;
            }
            else
            {
                if (!((stream >> height) && (height > 0)))
                {
                    std::cout << "Warning! Invalid value of '" << CNS_TAG_HEIGHT
                              << "' tag encountered (or could not convert to integer)." << std::endl;
                    std::cout << "Value of '" << CNS_TAG_HEIGHT << "' tag should be an integer >=0" << std::endl;
                    std::cout << "Continue reading XML and hope correct value of '" << CNS_TAG_HEIGHT
                              << "' tag will be encountered later..." << std::endl;
                }
                else
                    hasHeight = true;
            }
        }
        else if (value == CNS_TAG_WIDTH)
        {
            if (hasWidth)
            {
                std::cout << "Warning! Duplicate '" << CNS_TAG_WIDTH << "' encountered." << std::endl;
                std::cout << "Only first value of '" << CNS_TAG_WIDTH << "' =" << width << "will be used." << std::endl;
            }
            else
            {
                if (!((stream >> width) && (width > 0)))
                {
                    std::cout << "Warning! Invalid value of '" << CNS_TAG_WIDTH
                              << "' tag encountered (or could not convert to integer)." << std::endl;
                    std::cout << "Value of '" << CNS_TAG_WIDTH << "' tag should be an integer AND >0" << std::endl;
                    std::cout << "Continue reading XML and hope correct value of '" << CNS_TAG_WIDTH
                              << "' tag will be encountered later..." << std::endl;

                }
                else
                    hasWidth = true;
            }
        }
        else if (value == CNS_TAG_GRID)
        {
            int rowiter(0), grid_i(0), grid_j(0);
            hasGrid = true;
            if (!(hasHeight && hasWidth))
            {
                std::cout << "Error! No '" << CNS_TAG_WIDTH << "' tag or '" << CNS_TAG_HEIGHT << "' tag before '"
                          << CNS_TAG_GRID << "'tag encountered!" << std::endl;
                return false;
            }
            element = mapnode->FirstChildElement();
            while (grid_i < height)
            {
                if (!element)
                {
                    std::cout << "Error! Not enough '" << CNS_TAG_ROW << "' tags inside '" << CNS_TAG_GRID << "' tag."
                              << std::endl;
                    std::cout << "Number of '" << CNS_TAG_ROW
                              << "' tags should be equal (or greater) than the value of '" << CNS_TAG_HEIGHT
                              << "' tag which is " << height << std::endl;
                    return false;
                }
                std::string str = element->GetText();
                std::vector<std::string> elems;
                std::stringstream ss(str);
                std::string item;
                while (std::getline(ss, item, ' '))
                    elems.push_back(item);
                rowiter = grid_j = 0;
                int val;
                if (elems.size() > 0)
                    for (grid_j = 0; grid_j < width; ++grid_j)
                    {
                        if (grid_j == elems.size())
                            break;
                        stream.str("");
                        stream.clear();
                        stream << elems[grid_j];
                        stream >> val;
                        grid[grid_i][grid_j] = val;
                    }

                if (grid_j != width)
                {
                    std::cout << "Invalid value on " << CNS_TAG_GRID << " in the " << grid_i + 1 << " " << CNS_TAG_ROW
                              << std::endl;
                    return false;
                }
                ++grid_i;
                element = element->NextSiblingElement();
            }
        }
    }
    if (!hasGrid) {
        std::cout << "Error! There is no tag 'grid' in xml-file!\n";
        return false;
    }
    return true;
}

void Map::generate_moves()
{
    std::vector<Step> moves;
    valid_moves.resize(height);
    for(int i = 0; i < height; i++)
        valid_moves[i].resize(width);
    if(CN_K == 2)
        moves = {{0,1}, {1,0}, {-1,0},  {0,-1}};
    else if(CN_K == 3)
        moves = {{0,1}, {1,1}, {1,0},  {1,-1},  {0,-1},  {-1,-1}, {-1,0}, {-1,1}};
    else if(CN_K == 4)
        moves = {{0,1}, {1,1}, {1,0},  {1,-1},  {0,-1},  {-1,-1}, {-1,0}, {-1,1},
                 {1,2}, {2,1}, {2,-1}, {1,-2}, {-1,-2}, {-2,-1}, {-2,1},  {-1,2}};
    else
        moves = {{0,1},   {1,1},   {1,0},   {1,-1},  {0,-1},  {-1,-1}, {-1,0}, {-1,1},
                 {1,2},   {2,1},   {2,-1},  {1,-2},  {-1,-2}, {-2,-1}, {-2,1}, {-1,2},
                 {1,3},   {2,3},   {3,2},   {3,1},   {3,-1},  {3,-2},  {2,-3}, {1,-3},
                 {-1,-3}, {-2,-3}, {-3,-2}, {-3,-1}, {-3,1},  {-3,2},  {-2,3}, {-1,3}};
    for(int i = 0; i < moves.size(); i++)
        moves[i].cost = sqrt(pow(moves[i].i, 2) + pow(moves[i].j, 2));
    for(int i = 0; i < height; i++)
        for(int j = 0; j < width; j++)
        {
            std::vector<bool> valid(moves.size(), true);
            if(CN_K == 2)
            {
                for(int k = 0; k < 4; k++)
                    if(!cell_on_grid(i + moves[k].i, j + moves[k].j) || cell_is_obstacle(i + moves[k].i, j + moves[k].j))
                        valid[k] = false;
            }
            else if(CN_K == 3)
            {
                for(int k = 0; k < 8; k += 2)
                    if(!cell_on_grid(i + moves[k].i, j + moves[k].j) || cell_is_obstacle(i + moves[k].i, j + moves[k].j))
                    {
                        valid[k-1 < 0 ? 7 : k - 1] = false; valid[k] = false; valid[k + 1] = false;
                    }
                for(int k = 1; k < 8; k += 2)
                    if(!cell_on_grid(i + moves[k].i, j + moves[k].j) || cell_is_obstacle(i + moves[k].i, j + moves[k].j))
                        valid[k] = false;
            }
            else if(CN_K == 4)
            {
                for(int k = 0; k < 8; k += 2)
                    if(!cell_on_grid(i + moves[k].i, j + moves[k].j) || cell_is_obstacle(i + moves[k].i, j + moves[k].j))
                    {
                        if(k == 0)
                        {
                            valid[7]  = false;
                            valid[15] = false;
                        }
                        else
                        {
                            valid[k - 1] = false;
                            valid[k + 7] = false;
                        }
                        valid[k]     = false;
                        valid[k + 1] = false;
                        valid[k + 8] = false;
                    }
                for(int k = 1; k < 8; k += 2)
                    if(!cell_on_grid(i + moves[k].i, j + moves[k].j) || cell_is_obstacle(i + moves[k].i, j + moves[k].j))
                    {
                        valid[k]     = false;
                        valid[k + 7] = false;
                        valid[k + 8] = false;
                    }
                for(int k = 8; k < 16; k++)
                    if(!cell_on_grid(i + moves[k].i, j + moves[k].j) || cell_is_obstacle(i + moves[k].i, j + moves[k].j))
                        valid[k] = false;
            }
            else
            {
                for(int k = 0; k < 8; k += 2)
                    if(!cell_on_grid(i + moves[k].i, j + moves[k].j) || cell_is_obstacle(i + moves[k].i, j + moves[k].j))
                    {
                        if(k == 0)
                        {
                            valid[7]  = false;
                            valid[15] = false;
                            valid[30] = false;
                            valid[31] = false;
                        }
                        else
                        {
                            valid[k - 1]    = false;
                            valid[k + 7]    = false;
                            valid[k*2 + 14] = false;
                            valid[k*2 + 15] = false;
                        }
                        valid[k]        = false;
                        valid[k + 1]    = false;
                        valid[k + 8]    = false;
                        valid[k*2 + 16] = false;
                        valid[k*2 + 17] = false;
                    }
                for(int k = 1; k < 8; k += 2)
                    if(!cell_on_grid(i + moves[k].i, j + moves[k].j) || cell_is_obstacle(i + moves[k].i, j + moves[k].j))
                    {
                        valid[k]     = false;
                        valid[k + 7] = false;
                        valid[k + 8] = false;
                        for(int l = 14; l < 18; l++)
                            valid[k*2 + l] = false;
                    }
                for(int k = 8; k < 16; k++)
                    if(!cell_on_grid(i + moves[k].i, j + moves[k].j) || cell_is_obstacle(i + moves[k].i, j + moves[k].j))
                    {
                        valid[k]       = false;
                        valid[k*2]     = false;
                        valid[k*2 + 1] = false;
                    }
                for(int k = 16; k < 32; k++)
                    if(!cell_on_grid(i + moves[k].i, j + moves[k].j) || cell_is_obstacle(i + moves[k].i, j + moves[k].j))
                        valid[k]       = false;
            }

            std::vector<Step> v_moves = {};
            for(int k = 0; k < valid.size(); k++)
                if(valid[k])
                    v_moves.push_back(moves[k]);
            valid_moves[i][j] = v_moves;
        }
}

void Map::print_map()
{
    std::cout<<height<<"x"<<width<<std::endl;
    for(int i = 0; i < height; i++)
    {
        for(int j = 0; j < width; j++)
            std::cout<<grid[i][j]<<" ";
        std::cout<<std::endl;
    }
}

bool Map::cell_is_traversable(int i, int j) const
{
    return (grid[i][j] != CN_OBSTL);
}

bool Map::cell_is_obstacle(int i, int j) const
{
    return (grid[i][j] == CN_OBSTL);
}

bool Map::cell_on_grid(int i, int j) const
{
    return (i < height && i >= 0 && j < width && j >= 0);
}

std::vector<Step> Map::get_valid_moves(int i, int j) const
{
    return valid_moves[i][j];
}

int Map::get_value(int i, int j) const
{
    if(i < 0 || j < 0 || i >= height || j >= width)
        return -1;
    else
        return grid[i][j];
}
