from geometry_msgs.msg import Point
from math import floor


class Grid:
    # x is right
    # y is up
    def __init__(self, nb_row, nb_col, base, maximum):

        self.row = nb_row
        self.col = nb_col
        self.base = base
        self.maximum = maximum
        self.blocklengthX = (float(maximum.x - base.x))/ nb_col
        self.blocklengthY = (float(maximum.y - base.y))/ nb_row

    ''' The following is to convert from state 0 being in the bottom left to state 0 being in the top left '''
    def row_converter(self, curr_row, curr_col):
        new_col = curr_col
        new_row = self.row - curr_row - 1
        return new_row, new_col

    def cart2state(self, position):
        curr_row = 0
        curr_col = 0
        assert(position.x >= self.base.x and position.x <= self.maximum.x), "x position %r is out of bounds! It should be at least %r and at most %r. " % (position.x, self.base.x, self.maximum.x)
        assert(position.y >= self.base.y and position.y <= self.maximum.y), "y position %r is out of bounds! It should be at least %r and at most %r.  " % (position.y, self.base.y, self.maximum.y)
        curr_row = int((position.y - self.base.y)/self.blocklengthY)
        curr_col = int((position.x - self.base.x)/self.blocklengthX)
        assert(curr_col >= 0 and curr_col<= self.col), "x position %r is out of bounds! It should be at least %r and at most %r.  " % (curr_col, 0, self.col)
        assert(curr_row >= 0 and curr_row <= self.row), "y position %r is out of bounds! It should be at least %r and at most %r.  " % (curr_row, 0, self.row)
        # # The following is for returning a state where the 0 state is at the bottom left
        # state = floor(curr_col) + (floor(curr_row)*self.col)

        # The following is for changing the state numbering scheme from bottom left to top left
        new_row, new_col = self.row_converter(curr_row, curr_col)
        state = floor(new_col) + (floor(new_row)*self.col)
        return int(state)

    def state2cart(self, state):
        curr_col = state % self.col
        curr_row = state / self.col
        position = Point()
        position.z = self.base.z
        # # The following is for when the 0 state is at the bottom left
        # position.x = self.base.x + (self.blocklengthX/2.) + (curr_col * self.blocklengthX ) 
        # position.y = self.base.y + (self.blocklengthY/2.) + (curr_row * self.blocklengthY )
        
        # The following is for changing from top left back to bottom left
        new_row, new_col = self.row_converter(curr_row, curr_col)
        position.x = self.base.x + (self.blocklengthX/2.) + (new_col * self.blocklengthX ) 
        position.y = self.base.y + (self.blocklengthY/2.) + (new_row * self.blocklengthY )  
        return position

    def cart2state_list(self , position):
        res = list()
        for elem in position:
            res.append(self.cart2state(elem))
        return res
