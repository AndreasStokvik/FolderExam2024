#include "GameManager.h"
#include <memory>

int main() {
    std::unique_ptr<GameManager> game = std::make_unique<GameManager>();
    game->init();
    game->run();
    return 0;
}