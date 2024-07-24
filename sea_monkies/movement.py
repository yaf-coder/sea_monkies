#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import OverrideRCIn


class DanceNode(Node):
    _move_forward = False
    _move_back = False
    _move_left = False
    _move_right = False
    _move_up = False
    _move_down = False
    _turn_left = False
    _turn_right = False
    _step_counter = 0

    def __init__(self):
        super().__init__("dancing_node")

        self.command_pub = self.create_publisher(
            OverrideRCIn, "bluerov2/override_rc", 10
        )

        self.loop = self.create_timer(1.0, self._loop)

    def _set_neutral_all_channels(self):
        neutral = OverrideRCIn()
        neutral.channels = [1500] * 8
        self.command_pub.publish(neutral)

    def _loop(self):
        if self._step_counter > 60:
            self.destroy_node()
            return
        self._dance_moves()

        # See https://www.ardusub.com/developers/rc-input-and-output.html#rc-input
        commands = OverrideRCIn()
        commands.channels = [OverrideRCIn.CHAN_NOCHANGE] * 8
        if self._move_forward:
            commands.channels[4] = 1700
        elif self._move_back:
            commands.channels[4] = 1300
        if self._move_left:
            commands.channels[5] = 1300
        elif self._move_right:
            commands.channels[5] = 1700
        if self._move_up:
            commands.channels[2] = 1700
        elif self._move_down:
            commands.channels[2] = 1300
        if self._turn_left:
            commands.channels[3] = 1300
        elif self._turn_right:
            commands.channels[3] = 1700

        self.command_pub.publish(commands)

    def _dance_moves(self):
        self._step_counter += 1

        if self._step_counter < 5:
            self._move_forward = True
            self._move_back = False
        elif self._step_counter < 7:
            self._move_forward = False
            self._move_back = True
        elif self._step_counter < 10:
            self._move_forward = False
            self._move_back = False
            self._move_left = True
            self._move_right = False
        elif self._step_counter < 16:
            self._move_left = False
            self._move_right = True
        elif self._step_counter < 21:
            self._move_right = False
            self._move_left = False
            self._move_down = True
            self._move_up = False
        elif self._step_counter < 25:
            self._move_down = False
            self._move_up = True
        elif self._step_counter < 30:
            self._move_down = False
            self._move_up = False
            self._turn_left = True
            self._turn_right = False
            self._move_forward = True
        elif self._step_counter < 35:
            self._turn_left = False
            self._turn_right = True
        elif self._step_counter < 40:
            self._set_neutral_all_channels()
            # self._turn_right = False
            # self._turn_left = False
            # self._move_forward = False
            # self._move_back = False
            # self._move_left = False
            # self._move_right = False
            # self._move_down = False
            # self._move_up = False

    def destroy_node(self):
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    danceNode = DanceNode()

    try:
        rclpy.spin(danceNode)
    except KeyboardInterrupt:
        pass
    finally:
        danceNode.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()