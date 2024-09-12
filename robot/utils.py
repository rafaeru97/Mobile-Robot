def save_path_to_file(path, filename="path.txt"):
    with open(filename, 'w') as file:
        for point in path:
            file.write(f"{point}\n")

def load_path_from_file(filename="path.txt"):
    path = []
    with open(filename, 'r') as file:
        for line in file:
            point = tuple(map(int, line.strip().strip('()').split(', ')))
            path.append(point)
    return path