#include <iostream>
#include "map.h"
#include "task.h"
#include "cbs.h"
#include "xml_logger.h"
#include <fstream>

int main(int argc, const char *argv[])
{
    bool run_64x64_empty_grids = false;
    bool run_10x10_empty_grids = true;
    bool run_warehouse = false;
    int task_num = 1000;
    std::vector<int> agents_num = {4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
    std::string path = "instances/";
    if(run_64x64_empty_grids)
        path += "64x64_empty/";
    else if(run_10x10_empty_grids)
        path += "10x10_empty/";
    else if (run_warehouse)
        path += "warehouse/";
    if(run_64x64_empty_grids || run_10x10_empty_grids || run_warehouse)
    {
        Map map = Map();
        map.get_map((path + "map.xml").c_str());
        map.generate_moves();
        for(int k = 0; k < agents_num.size(); k++)
        {
            std::cout<< "Solving " << task_num << " tasks with " << agents_num[k] << " agents" << std::endl;
            for(int i = 0; i < task_num; i++)
            {
                Task task;
                task.get_task((path + std::to_string(agents_num[k]) + "/" + std::to_string(i) + "_task.xml").c_str());
                CBS cbs;
                Solution solution = cbs.find_solution(map, task);
                XML_logger logger;
                std::cout<< agents_num[k] << " " << i << " " << solution.time.count() << " " << solution.makespan << " " << solution.flowtime << std::endl;
                logger.get_log((path + std::to_string(agents_num[k]) + "/" + std::to_string(i) + "_task.xml").c_str());
                logger.write_to_log_summary(solution);
                logger.write_to_log_path(solution);
                logger.save_log();
            }
        }
    }
    else if(argc > 1)
    {
        Map map = Map();
        map.get_map(argv[1]);
        map.generate_moves();
        Task task;
        task.get_task(argv[2]);
        CBS cbs;
        Solution solution = cbs.find_solution(map, task);
        XML_logger logger;
        std::cout<<solution.time.count()<<" "<<solution.makespan<<" "<<solution.flowtime<<std::endl;
        logger.get_log(argv[2]);
        logger.write_to_log_summary(solution);
        logger.write_to_log_path(solution);
        logger.save_log();
    }
    return 0;
}
