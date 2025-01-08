import random

def create_random_nodes(num_nodes=10, range_min=0, range_max=100):
    nodes = []
    for i in range(num_nodes):
        node = {
            'id': i,
            'x': random.uniform(range_min, range_max),
            'y': random.uniform(range_min, range_max),
            'z': random.uniform(range_min, range_max)
        }
        nodes.append(node)
    return nodes

if __name__ == "__main__":
    random_nodes = create_random_nodes()
    print(random_nodes)