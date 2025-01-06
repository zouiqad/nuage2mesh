#include <iostream>
#include "core/Engine.h"


int main () {
    n2m::Engine* engine = new n2m::Engine ();

    if (!engine->init ()) {
        std::cout << "Failed to initialize engine" << std::endl;
        return 1;
    }

    engine->run ();

    delete engine;
    return 0;
}