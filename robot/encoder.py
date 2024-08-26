class Encoder:
    def __init__(self, pin_a, pin_b):
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.position = 0
        # Setup GPIO pins here

    def read(self):
        # Read the encoder value and update position
        pass

    def get_position(self):
        return self.position
