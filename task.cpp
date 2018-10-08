#include "task.h"
Task::Task()
{
    agents.clear();
}

bool Task::get_task(const char *FileName)
{
    tinyxml2::XMLElement *root = 0, *agent = 0;
    tinyxml2::XMLDocument doc;
    std::string value;
    std::stringstream stream;

    // Load XML File
    if (doc.LoadFile(FileName) != tinyxml2::XMLError::XML_SUCCESS)
    {
        std::cout << "Error opening XML file!" << std::endl;
        return false;
    }
    // Get ROOT element
    root = doc.FirstChildElement(CNS_TAG_ROOT);
    if (!root)
    {
        std::cout << "Error! No '" << CNS_TAG_ROOT << "' tag found in XML file!" << std::endl;
        return false;
    }


    for (agent = root->FirstChildElement(); agent; agent = agent->NextSiblingElement())
    {
        value = agent->Value();
        std::transform(value.begin(), value.end(), value.begin(), ::tolower);
        Agent a;
        stream.clear();
        stream << agent->Attribute(CNS_TAG_START_I);
        stream >> a.start_i;
        stream.clear();
        stream << agent->Attribute(CNS_TAG_START_J);
        stream >> a.start_j;
        stream.clear();
        stream << agent->Attribute(CNS_TAG_GOAL_I);
        stream >> a.goal_i;
        stream.clear();
        stream << agent->Attribute(CNS_TAG_GOAL_J);
        stream >> a.goal_j;
        a.id = agents.size();
        agents.push_back(a);

    }
}

Agent Task::get_agent(int id) const
{
    if(id >= 0 && id < agents.size())
        return agents[id];
    else
        return Agent();
}
