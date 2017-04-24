#!/usr/bin/python
import rospy

class CostCell(object):

    def __init__(self, x, y, v):
        self.x = x
        self.y = y
        self.val = v

        if 29 >= self.val:
            self.empty = True
        else:
            self.empty = False

    def set_h(self, goalX, goalY):
        """
        sets the H value to the manhattan distance to the goal
        :param goalX: The goal X position on the grid.
        :param goalY: The goal Y position on the grid.
        """
        self.h = (abs(goalX - self.x) + abs(goalY - self.y)) * 10 + self.get_val()

    def set_parent(self, pcell):
        """
        sets the parent of the cell and also the G value because that is done right after
        and then the F value because its just H + G and we already have H
        :param parentCell: The parent cell to this cell.
        """
        self.parent = pcell

        if (self.x == pcell.get_x_pos() or self.Ypos == pcell.get_y_pos()):
            self.g = pcell.get_g() + 10
        else:
            self.g = pcell.get_g() + 14
        self.f = self.h + self.g

    def is_not_in_list(self, mlist):
        for cell in mlist:
            if cell.get_x_pos() == self.x and cell.get_y_pos() == self.y:
                return False
        return True

    def is_empty(self):
        return self.empty

    def get_x_pos(self):
        return self.x

    def get_y_pos(self):
        return self.y

    def get_g(self):
        return self.g

    def get_h(self):
        return self.h

    def get_f(self):
        return self.f

    def get_parent(self):
        return self.parent

    def get_val(self):
        return self.val

    def is_unknown(self):
        return self.val < 0

    def set_val(self, val):
        """
        Set the square occupancy level.
        :param occupancyval: The new occupancy level.
        """
        self.val = val
        if 50 >= self.val:
            self.empty = True
        else:
            self.empty = False

    def __str__(self):
        return str(self.get_x_pos() )+ ' ' + str(self.get_y_pos()) + ' ' + str(self.is_empty()) + ' ' + str(self.is_unknown())

    def __repr__(self):
        return self.__str__()

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y
