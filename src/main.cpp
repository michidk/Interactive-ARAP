#include "gui.h"
#include <string>

int main(int argc, char *argv[])
{
    std::string path = "./data/decimated-knight.off"; // TODO: parse string from arguments
    auto *gui = new GUI(path);

    return 0;
}
