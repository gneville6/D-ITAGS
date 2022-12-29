class Patch:
    '''Patch for a single file'''

    def __init__(self):
        self.source_filename = None
        self.target_filename = None
        self.hunks = []

    def apply(self):
        pass
