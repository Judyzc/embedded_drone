#include "complementary_filter.h"
#include "controller.h"

void controller_update(attitude_t *att, controller_cmd_t *control_cmd) {
    control_cmd->timestamp = att->timestamp;
    control_cmd->cmd = 0.0f;
}