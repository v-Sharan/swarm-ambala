import os,yaml,csv,time
from pathlib import Path

def log(msg, LOG_FILE=None):

    if LOG_FILE is not None:
        with open(LOG_FILE, "a", encoding="utf-8") as f:
            f.write(msg + "\n")
            f.flush()

def define_log_path(log_path):
    cwd = os.getcwd()
    print("cwd", cwd)
    if log_path is not None:
    # remove leading / or \ so it becomes relative
        clean_log_path = log_path.lstrip("/\\")

        LOG_DIR = Path(cwd) / clean_log_path
        LOG_DIR.mkdir(parents=True, exist_ok=True)

        LOG_FILE = LOG_DIR / "swarm_server.log"
        log_path = str(LOG_FILE)

        print("Final log file path:", log_path)
        
        return log_path
    
def read_origin(filepath):
    print("Reading YAML from:", filepath)
    with open(filepath) as f:
        data = yaml.safe_load(f)

    origin = data.get("origin")

    # If the origin is provided as a string "(lat, lon)"
    if isinstance(origin, str):
        origin = origin.strip("()")
        lat, lon = origin.split(",")
        origin = (float(lat), float(lon))

    return origin

def read_specific_line(csv_file_path, line_number):
    csv_cache = {}
    goal = []

    # Check if we have already read this entire CSV into memory
    if csv_file_path not in csv_cache:
        try:
            with open(csv_file_path, "rt") as file:
                reader = csv.reader(file)
                # Read all lines into cache at once: [[float(x), float(y)], ...]
                csv_cache[csv_file_path] = [
                    [float(row[0]), float(row[1])] for row in reader if row
                ]
        except Exception as e:
            print(f"Error caching CSV {csv_file_path}: {e}")
            return [], csv_cache

    # Safely return the requested line from the cache (0-indexed)
    try:
        # The original code used a 0-based index but skipped `line_number` rows, effectively making it 0-indexed memory access
        target_line = csv_cache[csv_file_path][line_number]
        goal.append((target_line[0], target_line[1]))
    except IndexError:
        print(f"Error: Line {line_number} not found in {csv_file_path}")

    return goal,csv_cache[csv_file_path]
