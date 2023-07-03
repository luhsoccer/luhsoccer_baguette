#include "baguette.hpp"
#include "exception_handler/exception_handler.hpp"

int main() {
    luhsoccer::exception_handler::setTerminateHandler();
    luhsoccer::baguette::TheBaguette baguette;
    baguette.load();
    baguette.run();
    return 0;
}
