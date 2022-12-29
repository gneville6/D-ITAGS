class Hunk:
    '''Parsed hunk data container (hunk starts with @@ -R +R @@)'''

    def __init__(self):
        self.source_line_start = None
        self.source_line_count = None
        self.target_line_start = None
        self.target_line_count = None
        self.text = []

    def apply(self, original_text):
        pass
