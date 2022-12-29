#include "grstapse/task_allocation/itags/itags_v1_to_v2_problem_converter.hpp"

namespace grstapse
{

    void ItagsV1ToV2ProblemConverter::convertProblem(std::string filePath)
    {
        std::ifstream fin(filePath);
        nlohmann::json j;
        fin >> j;
        std::cout << j << std::endl;

    }

    void ItagsV1ToV2ProblemConverter::convertAllFilesInSubfolder(std::string filePath, std::string fileName) {}

}  // namespace grstapse