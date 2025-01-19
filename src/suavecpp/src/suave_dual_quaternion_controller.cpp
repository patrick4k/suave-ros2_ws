#include "start.h"
#include "nodes/SuaveDualQuaternionController.h"

int main(int argc, char **argv)
{
    return start<SuaveDualQuaternionController>(argc, argv);
}
