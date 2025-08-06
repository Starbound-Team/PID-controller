class DataLogger:
    def __init__(self, log_file):
        self.log_file = log_file
        self.data = []

    def log(self, timestamp, data_point):
        self.data.append((timestamp, data_point))
        with open(self.log_file, 'a') as f:
            f.write(f"{timestamp}, {data_point}\n")

    def get_data(self):
        return self.data

    def clear_log(self):
        self.data = []
        with open(self.log_file, 'w') as f:
            f.truncate()