class Area:
    
    def __init__(self, x, y, w, h):
        self.height = h
        self.width = w
        self.x = x
        self.y = y
    
    def set_position(self, x, y):
        self.x = x
        self.y = y
    
    def set_shape(self, w, h):
        self.width = w
        self.height = h
    
    def shape(self):
        return self.width,self.height
    
    def corners(self):
        return self.x-self.width/2, self.x+self.width/2, self.y-self.height/2, self.y+self.height/2
    
    def get_real_coordinate(self, x_loc, y_loc):
        return self.x-self.width/2 + x_loc, self.y-self.height/2+ y_loc