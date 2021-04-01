#! /usr/bin/env/ python2

import rospy

class ColorMixer(object):
    ''' Finds the color mix between two colors.
        color_0 and color_1 are chosen from:
            {'red', 'green', 'blue'}. '''
    def __init__(self, color_0, color_1):
        colors = ('red', 'green', 'blue')
        if not color_0 in colors or not color_1 in colors:
            rospy.logerr("Choose a color from ('red', 'green', 'blue')")
            sys.exit()

        self._c0 = self.__determine_input(color_0)
        self._c1 = self.__determine_input(color_1)
        self.last_c = []

    def __determine_input(self, color):
        if color == 'red':
            return [255, 0, 0]
        elif color == 'green':
            return [0, 255, 0]
        elif color == 'blue':
            return [0, 0, 255]
        else:
            rospy.logerr("Choose a color from ('red', 'green', 'blue')")
            sys.exit()

    def __check_mixing_value(self, val):
        if not (0 <= val <= 1):
            rospy.logerr("get_color value must be between [0-1]")
            sys.exit()

    def get_color(self, val):
        ''' Input is a double on [0-1] where:
            0 maps to c_0 = 255 and c_1 = 0,
            1 maps to c_0 = 0 and c_1 = 255. '''
        self.__check_mixing_value(val)
        self.last_c = [val*(self._c1[j] - self._c0[j]) + self._c0[j] for j in range(3)]
        return self.last_c

    def get_color_norm(self, val):
        self.__check_mixing_value(val)
        self.last_c = [val*(self._c1[j] - self._c0[j]) + self._c0[j] for j in range(3)]
        self.last_c[:] = [x/255 for x in self.last_c]
        return self.last_c