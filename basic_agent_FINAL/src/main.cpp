#include "core/Agent.h"

int main() {
  Agent agent("127.0.0.1", 30000);
  agent.run();
  return 0;
}
