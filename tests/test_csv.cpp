#include <iostream>
#include "../mechanics/utilities/parser.hpp"

int main()
{
    std::ifstream f("../data/test_csv.csv");
    aria::csv::CsvParser parser(f);

    for (auto &row : parser)
    {
        for (auto &field : row)
        {
            std::cout << std::stoi(field) << " | ";
        }
        std::cout << std::endl;
    }
    std::cout << 3%6 << std::endl;
    std::cout << int (36/6) << std::endl;
    std::cout << int (41/6) << std::endl;

}