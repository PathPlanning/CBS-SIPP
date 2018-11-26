#include <iostream>
#include "map.h"
#include "task.h"
#include "cbs.h"
#include "xml_logger.h"
#include <fstream>

int main(int argc, const char *argv[])
{
    bool run_empty_grids = false;
    bool run_dao = false;
    int task_num = 1000;
    std::vector<int> agents_num = {25};
    std::string path = "instances/";
    if(run_empty_grids)
        path += "10x10/";
    else if (run_dao)
        path += "den520d/";
    if(run_empty_grids || run_dao)
    {
        Map map = Map();
        map.get_map((path + "map.xml").c_str());
        map.generate_moves();
        for(int k = 0; k < agents_num.size(); k++)
            for(int i = 0; i < task_num; i++)
            {
                Task task;
                task.get_task((path + std::to_string(agents_num[k]) + "/" + std::to_string(i) + "_task.xml").c_str());
                CBS cbs;
                Solution solution = cbs.find_solution(map, task);
                XML_logger logger;
                std::cout<<agents_num[k]<<" "<<i<<" "<<solution.time.count()<<" "<<solution.makespan<<" "<<solution.flowtime<<" "
                        <<solution.initial_cost<<" "<<solution.expanded<<" "<<solution.ll_expanded<<std::endl;
                logger.get_log((path + std::to_string(agents_num[k]) + "/" + std::to_string(i) + "_task.xml").c_str());
                logger.write_to_log_summary(solution);
                logger.write_to_log_path(solution);
                logger.save_log();
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
        std::cout<<solution.time.count()<<" "<<solution.makespan<<" "<<solution.flowtime<<" "<<solution.initial_cost<<" "<<solution.expanded<<" "<<solution.ll_expanded<<" "<<solution.ll_searches<<std::endl;
        logger.get_log(argv[1]);
        logger.write_to_log_summary(solution);
        logger.write_to_log_path(solution);
        logger.save_log();
    }
    return 0;
}
